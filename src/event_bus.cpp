#include "raptor_midi_io/event_bus.hpp"

#include <chrono>
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
constexpr char kStatsTopic[] = "midi.io.stats";

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

std::uint64_t monotonic_time_ns() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}

json encode_packet_json(const MidiPacket& packet) {
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
        {"timestamp_ns", monotonic_time_ns()},
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
#endif
};

EventBus::EventBus(std::string events_endpoint)
    : events_endpoint_(std::move(events_endpoint)), impl_(std::make_unique<Impl>()) {
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

    if (zmq_bind(impl_->publisher, events_endpoint_.c_str()) != 0) {
        spdlog::error("zmq_bind failed for {}: {}", events_endpoint_, zmq_strerror(zmq_errno()));
        zmq_close(impl_->publisher);
        zmq_ctx_term(impl_->context);
        impl_->publisher = nullptr;
        impl_->context = nullptr;
        return;
    }
#endif
}

EventBus::~EventBus() {
#if RAPTOR_MIDI_IO_HAS_ZEROMQ
    if (impl_) {
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
    if (impl_ && impl_->publisher != nullptr) {
        const auto json = encode_packet_json(packet).dump();
        const auto send_topic = zmq_send(impl_->publisher, kTopic, sizeof(kTopic) - 1, ZMQ_SNDMORE);
        const auto send_payload = zmq_send(impl_->publisher, json.data(), json.size(), 0);

        if (send_topic >= 0 && send_payload >= 0) {
            spdlog::trace("publish endpoint={} seq={} source_kind={} bytes={}", events_endpoint_, packet.sequence, packet.source_kind, packet.bytes.size());
            return;
        }

        spdlog::error("ZeroMQ publish failed on {}: {}", events_endpoint_, zmq_strerror(zmq_errno()));
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
        "publish stats endpoint={} spi_invalid_port_drops_total={} bus_queue_dropped_events_total={} usb_queue_dropped_events_total={}",
        events_endpoint_,
        stats.spi_invalid_port_drops_total,
        stats.bus_queue_dropped_events_total,
        stats.usb_queue_dropped_events_total);
}
}  // namespace raptor::midi_io