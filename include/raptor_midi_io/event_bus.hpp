#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace raptor::midi_io {

struct MidiPacket {
    std::string module_id;
    std::string source_kind {"spi-module"};
    std::string controller_id;
    std::string device_name;
    std::string spi_device;
    std::uint32_t spi_speed_hz {0};
    std::uint8_t spi_mode {0};
    int chip_select_gpio {-1};
    int handshake_gpio {-1};
    bool handshake_active_low {false};
    std::size_t module_port_count {0};
    std::size_t module_first_global_port {0};
    std::size_t module_last_global_port {0};
    std::size_t local_port {0};
    std::size_t global_port {0};
    std::uint64_t timestamp_ns {0};
    std::vector<std::uint8_t> bytes;
    std::uint64_t sequence {0};
};

struct MidiIoStats {
    // Drops of decoded SPI frames where local_port is outside the module's configured port range.
    std::uint64_t spi_invalid_port_drops_total {0};

    // Drops due to internal queue overflows (data arrived but could not be queued for publish).
    std::uint64_t bus_queue_dropped_events_total {0};
    std::uint64_t usb_queue_dropped_events_total {0};

    // Downstream SPI TX health.
    std::uint64_t output_queue_dropped_events_total {0};
    std::uint64_t output_sent_events_total {0};
    std::uint64_t output_failed_events_total {0};
};

class EventBus {
public:
    explicit EventBus(std::string events_endpoint, std::string realtime_events_endpoint = {});
    ~EventBus();

    EventBus(const EventBus&) = delete;
    EventBus& operator=(const EventBus&) = delete;
    EventBus(EventBus&&) noexcept;
    EventBus& operator=(EventBus&&) noexcept;

    void publish(const MidiPacket& packet);
    void publish_stats(const MidiIoStats& stats);

private:
    struct Impl;
    std::string events_endpoint_;
    std::string realtime_events_endpoint_;
    std::unique_ptr<Impl> impl_;
};

}  // namespace raptor::midi_io
