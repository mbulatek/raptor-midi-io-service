#pragma once

#include "raptor_midi_io/config.hpp"
#include "raptor_midi_io/event_bus.hpp"

#include <atomic>
#include <memory>
#include <string>
#include <vector>

namespace raptor::midi_io {

struct BusIoMetrics {
    std::string bus_key;
    std::size_t module_count {0};
    std::size_t queue_depth {0};
    std::size_t queue_capacity {0};
    std::size_t queue_high_watermark {0};
    std::uint64_t dropped_events {0};
    std::size_t output_queue_depth {0};
    std::size_t output_queue_capacity {0};
    std::size_t output_queue_high_watermark {0};
    std::uint64_t output_dropped_events {0};
    std::uint64_t output_sent_events {0};
    std::uint64_t output_failed_events {0};
};

struct IoMetrics {
    std::uint64_t dropped_events_total {0};
    std::uint64_t output_dropped_events_total {0};
    std::uint64_t output_sent_events_total {0};
    std::uint64_t output_failed_events_total {0};
    std::uint64_t route_forwarded_events_total {0};
    std::uint64_t route_dropped_events_total {0};
    std::size_t queue_capacity_per_bus {0};
    std::size_t warning_threshold_percent {0};
    std::size_t route_count {0};
    std::string active_route_id;
    std::vector<BusIoMetrics> buses;
};

struct MidiRouteConfig {
    std::string id;
    int midi_in_port {-1};      // <=0 means ANY
    int midi_in_channel {0};    // <=0 means ANY
    int midi_out_port {-1};     // must be >0 when enabled
    int midi_out_channel {1};   // 1..16
    bool enabled {true};
};

class IoLoop {
public:
    IoLoop(const ServiceConfig& config,
           std::atomic<std::uint64_t>& published_packets,
           std::atomic<std::uint64_t>& sequence_counter);
    ~IoLoop();

    IoLoop(const IoLoop&) = delete;
    IoLoop& operator=(const IoLoop&) = delete;
    IoLoop(IoLoop&&) noexcept;
    IoLoop& operator=(IoLoop&&) noexcept;

    void start();
    void stop();
    void enqueue(MidiPacket packet);
    bool send_midi(std::size_t global_port, std::vector<std::uint8_t> bytes, std::string& error);
    bool upsert_route(MidiRouteConfig route, std::string& error);
    bool remove_route(const std::string& route_id, std::string& error);
    bool set_active_route(std::string route_id, std::string& error);
    IoMetrics snapshot() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace raptor::midi_io
