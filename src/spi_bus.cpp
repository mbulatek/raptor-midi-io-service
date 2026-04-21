#include "raptor_midi_io/spi_bus.hpp"

#include <algorithm>
#include <atomic>
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
constexpr std::size_t kEspBridgeFrameBytes = 96;

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

struct CachedSpiFd {
    int fd {-1};
    std::uint8_t mode {0};
    std::uint8_t bits_per_word {8};
    std::uint32_t speed_hz {0};
    bool configured {false};

    ~CachedSpiFd() {
        if (fd >= 0) {
            ::close(fd);
            fd = -1;
        }
    }
};

using SpiFdCache = std::unordered_map<std::string, CachedSpiFd>;

SpiFdCache& thread_spi_fd_cache() {
    static thread_local SpiFdCache cache;
    return cache;
}

void invalidate_cached_spi_fd(const std::string& spi_device) {
    auto& cache = thread_spi_fd_cache();
    const auto it = cache.find(spi_device);
    if (it == cache.end()) {
        return;
    }
    if (it->second.fd >= 0) {
        ::close(it->second.fd);
    }
    cache.erase(it);
}

bool acquire_cached_spi_fd(const ModuleConfig& module,
                           const std::uint8_t mode,
                           const std::uint8_t bits_per_word,
                           const std::uint32_t speed_hz,
                           int& fd_out) {
    auto& cache = thread_spi_fd_cache();
    auto& handle = cache[module.spi_device];

    if (handle.fd < 0) {
        handle.fd = ::open(module.spi_device.c_str(), O_RDWR);
        if (handle.fd < 0) {
            spdlog::warn("failed to open SPI device {}: {}", module.spi_device, std::strerror(errno));
            return false;
        }
        handle.configured = false;
    }

    if (!handle.configured || handle.mode != mode || handle.bits_per_word != bits_per_word || handle.speed_hz != speed_hz) {
        if (::ioctl(handle.fd, SPI_IOC_WR_MODE, &mode) < 0 ||
            ::ioctl(handle.fd, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word) < 0 ||
            ::ioctl(handle.fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz) < 0) {
            spdlog::debug(
                "spi cfg failed dev={} mode={} bits={} speed_hz={} err={}",
                module.spi_device,
                static_cast<int>(mode),
                static_cast<int>(bits_per_word),
                speed_hz,
                std::strerror(errno));
            spdlog::warn("failed to configure SPI device {}: {}", module.spi_device, std::strerror(errno));
            invalidate_cached_spi_fd(module.spi_device);
            return false;
        }

        handle.mode = mode;
        handle.bits_per_word = bits_per_word;
        handle.speed_hz = speed_hz;
        handle.configured = true;
    }

    fd_out = handle.fd;
    return true;
}

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

        const auto local_port =
            (payload.size() > 20 && payload[20] > 0) ? static_cast<std::size_t>(payload[20]) : 1;

        return SpiReadResult {
            .local_port = local_port,
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

bool transfer_frame(const ModuleConfig& module, const std::vector<std::uint8_t>& tx, std::vector<std::uint8_t>& rx) {
    ChipSelectGuard chip_select {module.chip_select_gpio};
    if (!chip_select.ready()) {
        spdlog::warn("failed to prepare chip-select GPIO {}", module.chip_select_gpio);
        return false;
    }

    std::uint8_t mode = module.spi_mode;
    if (module.chip_select_gpio >= 0) {
        mode |= SPI_NO_CS;
    }
    std::uint8_t bits_per_word = 8;
    std::uint32_t speed = module.spi_speed_hz;

    int fd = -1;
    if (!acquire_cached_spi_fd(module, mode, bits_per_word, speed, fd)) {
        return false;
    }

    ::spi_ioc_transfer transfer {};
    transfer.tx_buf = reinterpret_cast<unsigned long>(tx.data());
    transfer.rx_buf = reinterpret_cast<unsigned long>(rx.data());
    transfer.len = static_cast<std::uint32_t>(rx.size());
    transfer.speed_hz = speed;
    transfer.bits_per_word = bits_per_word;

    if (!chip_select.set_asserted(true)) {
        spdlog::warn("failed to assert chip-select GPIO {}", module.chip_select_gpio);
        return false;
    }

    const int result = ::ioctl(fd, SPI_IOC_MESSAGE(1), &transfer);
    (void)chip_select.set_asserted(false);

    if (result < 0) {
        spdlog::warn("SPI transfer failed on {}: {}", module.spi_device, std::strerror(errno));
        invalidate_cached_spi_fd(module.spi_device);
        return false;
    }
    return true;
}

#endif

}  // namespace

SpiReadResult SpiBus::read_packet(const ModuleConfig& module) {
#if defined(__linux__)
    const auto frame_size = std::max<std::size_t>(module.max_frame_bytes, kEspBridgeFrameBytes);
    std::vector<std::uint8_t> tx(frame_size, 0x00);
    std::vector<std::uint8_t> rx(frame_size, 0x00);
    spdlog::trace(
        "spi read module={} dev={} speed_hz={} mode={} frame_bytes={} cs_gpio={}",
        module.id,
        module.spi_device,
        module.spi_speed_hz,
        static_cast<int>(module.spi_mode),
        frame_size,
        module.chip_select_gpio);
    if (!transfer_frame(module, tx, rx)) {
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

bool SpiBus::write_packet(const ModuleConfig& module, std::size_t local_port, const std::vector<std::uint8_t>& bytes) {
    std::vector<SpiTxEvent> events;
    events.push_back(SpiTxEvent {.local_port = local_port, .bytes = bytes});
    return write_packets(module, events);
}

bool SpiBus::write_packets(const ModuleConfig& module, const std::vector<SpiTxEvent>& events) {
#if defined(__linux__)
    if (events.empty()) {
        return true;
    }

    for (const auto& event : events) {
        if (event.bytes.empty() || event.bytes.size() > 3) {
            spdlog::warn(
                "spi tx invalid midi bytes size module={} local_port={} bytes={}",
                module.id,
                event.local_port,
                event.bytes.size());
            return false;
        }
        if ((event.bytes[0] & 0x80) == 0) {
            spdlog::warn(
                "spi tx invalid status module={} local_port={} status=0x{:02X}",
                module.id,
                event.local_port,
                static_cast<unsigned int>(event.bytes[0]));
            return false;
        }
    }

    constexpr std::size_t kPacketSize = 24;
    constexpr std::uint32_t kMagicSingle = 0x4D494449u;  // "MIDI"
    constexpr std::uint32_t kMagicBatch = 0x4D494442u;   // "MIDB"
    constexpr std::size_t kBatchHeaderSize = 17;         // magic+seq+ts+count
    constexpr std::size_t kBatchEventSize = 5;           // local_port,size,status,data1,data2
    const std::size_t frame_size = std::max<std::size_t>(std::max<std::size_t>(module.max_frame_bytes, kPacketSize), kEspBridgeFrameBytes);

    const auto max_batch_events = (frame_size > kBatchHeaderSize)
                                      ? ((frame_size - kBatchHeaderSize) / kBatchEventSize)
                                      : 0;
    if (max_batch_events == 0) {
        spdlog::warn(
            "spi tx frame too small for batch module={} frame_bytes={} required>{}",
            module.id,
            frame_size,
            kBatchHeaderSize);
        return false;
    }

    if (events.size() > max_batch_events) {
        spdlog::warn(
            "spi tx batch too large module={} events={} max_events={}",
            module.id,
            events.size(),
            max_batch_events);
        return false;
    }

    const auto event_count = events.size();

    std::vector<std::uint8_t> tx(frame_size, 0x00);
    std::vector<std::uint8_t> rx(frame_size, 0x00);

    const auto write_u32_le = [&](const std::size_t offset, const std::uint32_t value) {
        if (offset + 4 > tx.size()) {
            return;
        }
        tx[offset + 0] = static_cast<std::uint8_t>((value >> 0) & 0xFFU);
        tx[offset + 1] = static_cast<std::uint8_t>((value >> 8) & 0xFFU);
        tx[offset + 2] = static_cast<std::uint8_t>((value >> 16) & 0xFFU);
        tx[offset + 3] = static_cast<std::uint8_t>((value >> 24) & 0xFFU);
    };
    const auto write_u64_le = [&](const std::size_t offset, const std::uint64_t value) {
        if (offset + 8 > tx.size()) {
            return;
        }
        for (std::size_t i = 0; i < 8; ++i) {
            tx[offset + i] = static_cast<std::uint8_t>((value >> (8 * i)) & 0xFFU);
        }
    };

    static std::atomic<std::uint32_t> tx_sequence {0};
    const auto sequence = tx_sequence.fetch_add(1, std::memory_order_relaxed) + 1;
    const auto now_us = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());

    if (event_count == 1) {
        const auto& event = events.front();
        write_u32_le(0, kMagicSingle);
        write_u32_le(4, sequence);
        write_u64_le(8, now_us);
        tx[16] = static_cast<std::uint8_t>(event.bytes.size());
        tx[17] = event.bytes[0];
        if (event.bytes.size() > 1) {
            tx[18] = event.bytes[1];
        }
        if (event.bytes.size() > 2) {
            tx[19] = event.bytes[2];
        }
        tx[20] = static_cast<std::uint8_t>(event.local_port & 0xFFU);

        spdlog::trace(
            "spi tx module={} local_port={} frame_bytes={} midi=[{}]",
            module.id,
            event.local_port,
            tx.size(),
            hex_dump(event.bytes));
    } else {
        write_u32_le(0, kMagicBatch);
        write_u32_le(4, sequence);
        write_u64_le(8, now_us);
        tx[16] = static_cast<std::uint8_t>(event_count & 0xFFU);

        std::size_t offset = kBatchHeaderSize;
        for (std::size_t i = 0; i < event_count; ++i) {
            const auto& event = events[i];
            tx[offset + 0] = static_cast<std::uint8_t>(event.local_port & 0xFFU);
            tx[offset + 1] = static_cast<std::uint8_t>(event.bytes.size() & 0xFFU);
            tx[offset + 2] = event.bytes[0];
            tx[offset + 3] = event.bytes.size() > 1 ? event.bytes[1] : 0x00;
            tx[offset + 4] = event.bytes.size() > 2 ? event.bytes[2] : 0x00;
            offset += kBatchEventSize;
        }

        spdlog::trace(
            "spi tx batch module={} events={} frame_bytes={}",
            module.id,
            event_count,
            tx.size());
    }

    return transfer_frame(module, tx, rx);
#else
    (void)module;
    (void)events;
    return true;
#endif
}

}  // namespace raptor::midi_io
