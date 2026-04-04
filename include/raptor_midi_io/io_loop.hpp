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
};

struct IoMetrics {
    std::uint64_t dropped_events_total {0};
    std::size_t queue_capacity_per_bus {0};
    std::size_t warning_threshold_percent {0};
    std::vector<BusIoMetrics> buses;
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
    IoMetrics snapshot() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace raptor::midi_io
