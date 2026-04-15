#include "raptor_midi_io/spi_bus.hpp"

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <chrono>
#include <unordered_map>
#include <vector>

#include <spdlog/spdlog.h>

#include <fmt/format.h>

#if defined(__linux__)
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#endif

#if defined(__linux__) && RAPTOR_MIDI_IO_HAS_GPIOD
#include <gpiod.h>
#endif

namespace raptor::midi_io {
namespace {

#if defined(__linux__)

static std::string hex_dump(const std::vector<std::uint8_t>& bytes) {
    std::string out;
    out.reserve(bytes.size() * 3);
    for (std::size_t i = 0; i < bytes.size(); ++i) {
        if (i) {
            out.push_back(' ');
        }
        out += fmt::format("{:02X}", bytes[i]);
    }
    return out;
}

static std::string hex_dump_prefix(const std::vector<std::uint8_t>& bytes, std::size_t max_bytes) {
    const std::size_t n = std::min<std::size_t>(max_bytes, bytes.size());
    std::string out;
    out.reserve(n * 3 + 6);
    for (std::size_t i = 0; i < n; ++i) {
        if (i) {
            out.push_back(' ');
        }
        out += fmt::format("{:02X}", bytes[i]);
    }
    if (bytes.size() > n) {
        out += " ...";
    }
    return out;
}

struct InvalidPortStats {
    std::uint64_t count {0};
    std::chrono::steady_clock::time_point last_log {};
};
SpiReadResult decode_payload(std::vector<std::uint8_t> payload) {
    if (payload.empty()) {
        return {};
    }

    // New ESP32 framing:
    //   u32 magic ("MIDI"), u32 sequence, u64 timestamp_us, u8 size, u8 status, u8 data1, u8 data2, ...
    // NOTE: No local_port in this packet; we currently map everything to port 1.
    constexpr std::uint32_t kMagic = 0x4D494449u;

    const auto read_u32_le = [&](std::size_t offset) -> std::uint32_t {
        if (offset + 4 > payload.size()) {
            return 0;
        }
        return (static_cast<std::uint32_t>(payload[offset + 0]) << 0) |
               (static_cast<std::uint32_t>(payload[offset + 1]) << 8) |
               (static_cast<std::uint32_t>(payload[offset + 2]) << 16) |
               (static_cast<std::uint32_t>(payload[offset + 3]) << 24);
    };

    const auto magic = read_u32_le(0);
    if (magic == kMagic) {
        // Minimum up to data2.
        if (payload.size() < 20) {
            return {};
        }

        const auto size = static_cast<std::size_t>(payload[16]);
        if (size < 1 || size > 3) {
            return {};
        }

        std::vector<std::uint8_t> bytes;
        bytes.reserve(size);
        bytes.push_back(payload[17]);
        if (size > 1) {
            bytes.push_back(payload[18]);
        }
        if (size > 2) {
            bytes.push_back(payload[19]);
        }

        return SpiReadResult {
            .local_port = 1,
            .bytes = std::move(bytes),
        };
    }

    // Legacy framing:
    //   u8 declared_size, u8 local_port, u8 midi_bytes...
    const auto declared_size = static_cast<std::size_t>(payload.front());
    if (declared_size > 0 && declared_size < payload.size()) {
        payload.erase(payload.begin());
        payload.resize(std::min(declared_size, payload.size()));
    } else {
        while (!payload.empty() && payload.back() == 0x00) {
            payload.pop_back();
        }
        if (!payload.empty()) {
            payload.erase(payload.begin());
        }
    }

    if (payload.size() < 2) {
        return {};
    }

    const auto local_port = static_cast<std::size_t>(payload.front());
    payload.erase(payload.begin());

    return SpiReadResult {
        .local_port = local_port,
        .bytes = std::move(payload),
    };
}

class ChipSelectGuard {
public:
    explicit ChipSelectGuard(int gpio) : gpio_(gpio) {
        if (gpio_ < 0) {
            return;
        }

#if RAPTOR_MIDI_IO_HAS_GPIOD
        chip_ = gpiod_chip_open("/dev/gpiochip0");
        if (chip_ == nullptr) {
            return;
        }

        settings_ = gpiod_line_settings_new();
        line_config_ = gpiod_line_config_new();
        request_config_ = gpiod_request_config_new();
        if (settings_ == nullptr || line_config_ == nullptr || request_config_ == nullptr) {
            return;
        }

        const unsigned int offsets[] = {static_cast<unsigned int>(gpio_)};
        gpiod_request_config_set_consumer(request_config_, "raptor-midi-io-service");

        const bool configured =
            gpiod_line_settings_set_direction(settings_, GPIOD_LINE_DIRECTION_OUTPUT) == 0 &&
            gpiod_line_settings_set_output_value(settings_, GPIOD_LINE_VALUE_ACTIVE) == 0 &&
            gpiod_line_config_add_line_settings(line_config_, offsets, 1, settings_) == 0;
        if (!configured) {
            return;
        }

        request_ = gpiod_chip_request_lines(chip_, request_config_, line_config_);
        if (request_ == nullptr) {
            return;
        }

        ready_ = true;
#endif
    }

    ~ChipSelectGuard() {
#if RAPTOR_MIDI_IO_HAS_GPIOD
        if (request_ != nullptr) {
            gpiod_line_request_set_value(request_, static_cast<unsigned int>(gpio_), GPIOD_LINE_VALUE_ACTIVE);
            gpiod_line_request_release(request_);
        }
        if (request_config_ != nullptr) {
            gpiod_request_config_free(request_config_);
        }
        if (line_config_ != nullptr) {
            gpiod_line_config_free(line_config_);
        }
        if (settings_ != nullptr) {
            gpiod_line_settings_free(settings_);
        }
        if (chip_ != nullptr) {
            gpiod_chip_close(chip_);
        }
#endif
    }

    bool ready() const {
        return gpio_ < 0 || ready_;
    }

    bool set_asserted(bool asserted) {
        if (gpio_ < 0) {
            return true;
        }
#if RAPTOR_MIDI_IO_HAS_GPIOD
        if (!ready_) {
            return false;
        }
        return gpiod_line_request_set_value(
                   request_,
                   static_cast<unsigned int>(gpio_),
                   asserted ? GPIOD_LINE_VALUE_INACTIVE : GPIOD_LINE_VALUE_ACTIVE) == 0;
#else
        return false;
#endif
    }

private:
    int gpio_ {-1};
    bool ready_ {false};
#if RAPTOR_MIDI_IO_HAS_GPIOD
    gpiod_chip* chip_ {nullptr};
    gpiod_line_request* request_ {nullptr};
    gpiod_line_settings* settings_ {nullptr};
    gpiod_line_config* line_config_ {nullptr};
    gpiod_request_config* request_config_ {nullptr};
#endif
};

#endif

}  // namespace

SpiReadResult SpiBus::read_packet(const ModuleConfig& module) {
#if defined(__linux__)
    ChipSelectGuard chip_select {module.chip_select_gpio};
    if (!chip_select.ready()) {
        spdlog::warn("failed to prepare chip-select GPIO {}", module.chip_select_gpio);
        return {};
    }

    const int fd = ::open(module.spi_device.c_str(), O_RDWR);
    if (fd < 0) {
        spdlog::warn("failed to open SPI device {}: {}", module.spi_device, std::strerror(errno));
        return {};
    }

    const auto close_fd = [&]() { ::close(fd); };

    std::uint8_t mode = module.spi_mode;

    if (module.chip_select_gpio >= 0) {

        mode |= SPI_NO_CS;

    }
    std::uint8_t bits_per_word = 8;
    std::uint32_t speed = module.spi_speed_hz;

    if (::ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0 ||
        ::ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word) < 0 ||
        ::ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        spdlog::debug("spi cfg failed dev={} mode={} bits={} speed_hz={} err={}", module.spi_device, static_cast<int>(mode), static_cast<int>(bits_per_word), speed, std::strerror(errno));
        spdlog::warn("failed to configure SPI device {}: {}", module.spi_device, std::strerror(errno));
        close_fd();
        return {};
    }

    std::vector<std::uint8_t> tx(module.max_frame_bytes, 0x00);
    spdlog::trace("spi read module={} dev={} speed_hz={} mode={} frame_bytes={} cs_gpio={}", module.id, module.spi_device, speed, static_cast<int>(mode), module.max_frame_bytes, module.chip_select_gpio);
    std::vector<std::uint8_t> rx(module.max_frame_bytes, 0x00);

    ::spi_ioc_transfer transfer {};
    transfer.tx_buf = reinterpret_cast<unsigned long>(tx.data());
    transfer.rx_buf = reinterpret_cast<unsigned long>(rx.data());
    transfer.len = static_cast<std::uint32_t>(rx.size());
    transfer.speed_hz = speed;
    transfer.bits_per_word = bits_per_word;

    if (!chip_select.set_asserted(true)) {
        spdlog::warn("failed to assert chip-select GPIO {}", module.chip_select_gpio);
        close_fd();
        return {};
    }

    const int result = ::ioctl(fd, SPI_IOC_MESSAGE(1), &transfer);
    (void)chip_select.set_asserted(false);
    close_fd();

    if (result < 0) {
        spdlog::warn("SPI transfer failed on {}: {}", module.spi_device, std::strerror(errno));
        return {};
    }

    const auto raw_rx = rx;
    auto decoded = decode_payload(std::move(rx));
    if (!decoded.bytes.empty() && (decoded.local_port < 1 || decoded.local_port > module.midi_port_count)) {
        // When SPI wiring is noisy, we can get garbage frames that still look non-empty after decode.
        // Logging the full raw buffer on every frame can block the service (journald backpressure).
        thread_local std::unordered_map<std::string, InvalidPortStats> stats;
        auto& st = stats[module.id];
        ++st.count;

        const auto now = std::chrono::steady_clock::now();
        const bool log_now =
            (st.count <= 5) ||
            (st.count % 200 == 0) ||
            (st.last_log.time_since_epoch().count() == 0) ||
            (now - st.last_log) > std::chrono::seconds(2);

        if (log_now) {
            st.last_log = now;
            spdlog::warn(
                "spi rx invalid local_port={} module={} configured_ports={} invalid_count={} raw_rx=[{}]",
                decoded.local_port,
                module.id,
                module.midi_port_count,
                st.count,
                hex_dump_prefix(raw_rx, 24));
        }
    }

    if (!decoded.bytes.empty()) {
        spdlog::trace("spi rx module={} local_port={} bytes={}", module.id, decoded.local_port, decoded.bytes.size());
    }
    return decoded;
#else
    return SpiReadResult {
        .local_port = 1,
        .bytes =
            {
                0x90,
                static_cast<std::uint8_t>(60 + (module.chip_select_gpio % 12)),
                0x7F,
            },
    };
#endif
}

}  // namespace raptor::midi_io
