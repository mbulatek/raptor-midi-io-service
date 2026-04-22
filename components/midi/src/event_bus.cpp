#include "raptor_midi_io/event_bus.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <string_view>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#if RAPTOR_MIDI_IO_HAS_ZEROMQ
#include <zmq.h>
#endif

namespace raptor::midi_io {

namespace {

using json = nlohmann::json;

constexpr char kSchemaVersion[] = "1.0";
constexpr char kServiceName[] = "raptor-midi-io-service";
constexpr char kTopic[] = "midi.packet";
constexpr char kRtTopic[] = "midi.packet.rt";
constexpr char kStatsTopic[] = "midi.io.stats";
constexpr std::uint32_t kRtMagic = 0x4D525431u;  // "MRT1"
constexpr std::uint16_t kRtVersion = 1u;

std::string format_bytes(const std::vector<std::uint8_t>& bytes) {
    std::ostringstream out;
    for (std::size_t i = 0; i < bytes.size(); ++i) {
        if (i != 0) {
            out << ' ';
        }
        out << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
            << static_cast<int>(bytes[i]);
    }
    return out.str();
}

void write_u16_le(std::vector<std::uint8_t>& out, const std::uint16_t value) {
    out.push_back(static_cast<std::uint8_t>(value & 0xFFu));
    out.push_back(static_cast<std::uint8_t>((value >> 8) & 0xFFu));
}

void write_u32_le(std::vector<std::uint8_t>& out, const std::uint32_t value) {
    out.push_back(static_cast<std::uint8_t>(value & 0xFFu));
    out.push_back(static_cast<std::uint8_t>((value >> 8) & 0xFFu));
    out.push_back(static_cast<std::uint8_t>((value >> 16) & 0xFFu));
    out.push_back(static_cast<std::uint8_t>((value >> 24) & 0xFFu));
}

void write_u64_le(std::vector<std::uint8_t>& out, const std::uint64_t value) {
    for (int i = 0; i < 8; ++i) {
        out.push_back(static_cast<std::uint8_t>((value >> (8 * i)) & 0xFFu));
    }
}

std::uint64_t monotonic_time_ns() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}

std::vector<std::uint8_t> encode_packet_rt_v1(const MidiPacket& packet, const std::uint64_t publish_ns) {
    const std::uint64_t source_ns = packet.timestamp_ns == 0 ? publish_ns : packet.timestamp_ns;
    const std::uint8_t size = static_cast<std::uint8_t>(std::min<std::size_t>(packet.bytes.size(), 3));
    std::vector<std::uint8_t> out;
    out.reserve(4 + 2 + 2 + 8 + 8 + 8 + 4 + 1 + 1 + 3 + 1);
    write_u32_le(out, kRtMagic);
    write_u16_le(out, kRtVersion);
    write_u16_le(out, 0u);  // reserved
    write_u64_le(out, packet.sequence);
    write_u64_le(out, source_ns);
    write_u64_le(out, publish_ns);
    write_u32_le(out, static_cast<std::uint32_t>(packet.global_port));
    out.push_back(size);
    out.push_back(static_cast<std::uint8_t>(packet.source_kind == "usb-midi-controller" ? 1 : 2));  // 1=usb,2=spi/other
    out.push_back(size > 0 ? packet.bytes[0] : 0u);
    out.push_back(size > 1 ? packet.bytes[1] : 0u);
    out.push_back(size > 2 ? packet.bytes[2] : 0u);
    out.push_back(0u);  // reserved
    return out;
}

json encode_packet_json(const MidiPacket& packet, const std::uint64_t publish_ns) {
    const auto ts_ns = packet.timestamp_ns == 0 ? monotonic_time_ns() : packet.timestamp_ns;
    json source = {
        {"source_kind", packet.source_kind},
        {"module_id", packet.module_id},
        {"controller_id", packet.controller_id},
        {"device_name", packet.device_name},
        {"spi_device", packet.spi_device},
        {"spi_speed_hz", packet.spi_speed_hz},
        {"spi_mode", packet.spi_mode},
        {"chip_select_gpio", packet.chip_select_gpio},
        {"handshake_gpio", packet.handshake_gpio},
        {"handshake_active_low", packet.handshake_active_low},
        {"module_port_count", packet.module_port_count},
        {"module_first_global_port", packet.module_first_global_port},
        {"module_last_global_port", packet.module_last_global_port},
        {"local_port", packet.local_port},
        {"global_port", packet.global_port},
    };

    return {
        {"schema_version", kSchemaVersion},
        {"service", kServiceName},
        {"type", "midi.packet"},
        {"sequence", packet.sequence},
        {"timestamp_ns", ts_ns},
        {"source_timestamp_ns", ts_ns},
        {"publish_timestamp_ns", publish_ns},
        {"source", source},
        {"midi",
         {
             {"size", packet.bytes.size()},
             {"bytes", packet.bytes},
             {"bytes_hex", format_bytes(packet.bytes)},
         }},
    };
}


json encode_stats_json(const MidiIoStats& stats) {
    return {
        {"schema_version", kSchemaVersion},
        {"service", kServiceName},
        {"type", "midi.io.stats"},
        {"timestamp_ns", monotonic_time_ns()},
        {"spi_invalid_port_drops_total", stats.spi_invalid_port_drops_total},
        {"bus_queue_dropped_events_total", stats.bus_queue_dropped_events_total},
        {"usb_queue_dropped_events_total", stats.usb_queue_dropped_events_total},
        {"output_queue_dropped_events_total", stats.output_queue_dropped_events_total},
        {"output_sent_events_total", stats.output_sent_events_total},
        {"output_failed_events_total", stats.output_failed_events_total},
    };
}
std::filesystem::path endpoint_directory(const std::string& endpoint) {
    constexpr std::string_view prefix {"ipc://"};
    if (!endpoint.starts_with(prefix)) {
        return {};
    }

    auto path = endpoint.substr(prefix.size());
    return std::filesystem::path {path}.parent_path();
}

}  // namespace

struct EventBus::Impl {
#if RAPTOR_MIDI_IO_HAS_ZEROMQ
    void* context {nullptr};
    void* publisher {nullptr};
    void* realtime_publisher {nullptr};
#endif
};

EventBus::EventBus(std::string events_endpoint, std::string realtime_events_endpoint)
    : events_endpoint_(std::move(events_endpoint)),
      realtime_events_endpoint_(std::move(realtime_events_endpoint)),
      impl_(std::make_unique<Impl>()) {
#if RAPTOR_MIDI_IO_HAS_ZEROMQ
    const auto directory = endpoint_directory(events_endpoint_);
    if (!directory.empty()) {
        std::error_code ec;
        std::filesystem::create_directories(directory, ec);
        if (ec) {
            spdlog::warn("failed to create ZeroMQ endpoint directory {}: {}", directory.string(), ec.message());
        }
    }

    impl_->context = zmq_ctx_new();
    if (impl_->context == nullptr) {
        spdlog::error("zmq_ctx_new failed for {}: {}", events_endpoint_, zmq_strerror(zmq_errno()));
        return;
    }

    impl_->publisher = zmq_socket(impl_->context, ZMQ_PUB);
    if (impl_->publisher == nullptr) {
        spdlog::error("zmq_socket(ZMQ_PUB) failed for {}: {}", events_endpoint_, zmq_strerror(zmq_errno()));
        zmq_ctx_term(impl_->context);
        impl_->context = nullptr;
        return;
    }

    constexpr int linger_ms = 0;
    (void)zmq_setsockopt(impl_->publisher, ZMQ_LINGER, &linger_ms, sizeof(linger_ms));

    if (!events_endpoint_.empty() && zmq_bind(impl_->publisher, events_endpoint_.c_str()) != 0) {
        spdlog::error("zmq_bind failed for {}: {}", events_endpoint_, zmq_strerror(zmq_errno()));
        zmq_close(impl_->publisher);
        zmq_ctx_term(impl_->context);
        impl_->publisher = nullptr;
        impl_->context = nullptr;
        return;
    }

    if (!realtime_events_endpoint_.empty()) {
        const auto rt_directory = endpoint_directory(realtime_events_endpoint_);
        if (!rt_directory.empty()) {
            std::error_code ec;
            std::filesystem::create_directories(rt_directory, ec);
            if (ec) {
                spdlog::warn("failed to create ZeroMQ rt endpoint directory {}: {}", rt_directory.string(), ec.message());
            }
        }

        impl_->realtime_publisher = zmq_socket(impl_->context, ZMQ_PUB);
        if (impl_->realtime_publisher == nullptr) {
            spdlog::error("zmq_socket(ZMQ_PUB) failed for realtime endpoint {}: {}", realtime_events_endpoint_, zmq_strerror(zmq_errno()));
            return;
        }

        constexpr int linger_ms = 0;
        (void)zmq_setsockopt(impl_->realtime_publisher, ZMQ_LINGER, &linger_ms, sizeof(linger_ms));

        if (zmq_bind(impl_->realtime_publisher, realtime_events_endpoint_.c_str()) != 0) {
            spdlog::error("zmq_bind failed for realtime endpoint {}: {}", realtime_events_endpoint_, zmq_strerror(zmq_errno()));
            zmq_close(impl_->realtime_publisher);
            impl_->realtime_publisher = nullptr;
        }
    }
#endif
}

EventBus::~EventBus() {
#if RAPTOR_MIDI_IO_HAS_ZEROMQ
    if (impl_) {
        if (impl_->realtime_publisher != nullptr) {
            zmq_close(impl_->realtime_publisher);
        }
        if (impl_->publisher != nullptr) {
            zmq_close(impl_->publisher);
        }
        if (impl_->context != nullptr) {
            zmq_ctx_term(impl_->context);
        }
    }
#endif
}

EventBus::EventBus(EventBus&&) noexcept = default;
EventBus& EventBus::operator=(EventBus&&) noexcept = default;

void EventBus::publish(const MidiPacket& packet) {
#if RAPTOR_MIDI_IO_HAS_ZEROMQ
    bool sent_any = false;
    const auto publish_ns = monotonic_time_ns();

    if (impl_ && impl_->realtime_publisher != nullptr) {
        const auto bin = encode_packet_rt_v1(packet, publish_ns);
        const auto send_topic = zmq_send(impl_->realtime_publisher, kRtTopic, sizeof(kRtTopic) - 1, ZMQ_SNDMORE);
        const auto send_payload = zmq_send(impl_->realtime_publisher, bin.data(), bin.size(), 0);
        if (send_topic >= 0 && send_payload >= 0) {
            sent_any = true;
        } else {
            spdlog::error("ZeroMQ realtime publish failed on {}: {}", realtime_events_endpoint_, zmq_strerror(zmq_errno()));
        }
    }

    if (impl_ && impl_->publisher != nullptr) {
        const auto json = encode_packet_json(packet, publish_ns).dump();
        const auto send_topic = zmq_send(impl_->publisher, kTopic, sizeof(kTopic) - 1, ZMQ_SNDMORE);
        const auto send_payload = zmq_send(impl_->publisher, json.data(), json.size(), 0);
        if (send_topic >= 0 && send_payload >= 0) {
            sent_any = true;
        } else {
            spdlog::error("ZeroMQ publish failed on {}: {}", events_endpoint_, zmq_strerror(zmq_errno()));
        }
    }

    if (sent_any) {
        spdlog::trace(
            "publish seq={} source_kind={} bytes={} endpoints=json:{} rt:{}",
            packet.sequence,
            packet.source_kind,
            packet.bytes.size(),
            events_endpoint_,
            realtime_events_endpoint_);
        return;
    }
#endif

    spdlog::debug(
        "publish endpoint={} seq={} source_kind={} module={} controller={} bytes={} transport={}",
        events_endpoint_,
        packet.sequence,
        packet.source_kind,
        packet.module_id,
        packet.controller_id,
        format_bytes(packet.bytes),
#if RAPTOR_MIDI_IO_HAS_ZEROMQ
        "fallback-log"
#else
        "stub"
#endif
    );
}


void EventBus::publish_stats(const MidiIoStats& stats) {
#if RAPTOR_MIDI_IO_HAS_ZEROMQ
    if (impl_ && impl_->publisher != nullptr) {
        const auto json = encode_stats_json(stats).dump();
        const auto send_topic = zmq_send(impl_->publisher, kStatsTopic, sizeof(kStatsTopic) - 1, ZMQ_SNDMORE);
        const auto send_payload = zmq_send(impl_->publisher, json.data(), json.size(), 0);

        if (send_topic >= 0 && send_payload >= 0) {
            return;
        }

        spdlog::error("ZeroMQ publish stats failed on {}: {}", events_endpoint_, zmq_strerror(zmq_errno()));
        return;
    }
#endif

    // When ZeroMQ isn't available, stats are only visible via logs.
    spdlog::trace(
        "publish stats endpoint={} spi_invalid_port_drops_total={} bus_queue_dropped_events_total={} usb_queue_dropped_events_total={} output_queue_dropped_events_total={} output_sent_events_total={} output_failed_events_total={}",
        events_endpoint_,
        stats.spi_invalid_port_drops_total,
        stats.bus_queue_dropped_events_total,
        stats.usb_queue_dropped_events_total,
        stats.output_queue_dropped_events_total,
        stats.output_sent_events_total,
        stats.output_failed_events_total);
}
}  // namespace raptor::midi_io
