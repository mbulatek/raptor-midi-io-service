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
    std::vector<std::uint8_t> bytes;
    std::uint64_t sequence {0};
};

class EventBus {
public:
    explicit EventBus(std::string events_endpoint);
    ~EventBus();

    EventBus(const EventBus&) = delete;
    EventBus& operator=(const EventBus&) = delete;
    EventBus(EventBus&&) noexcept;
    EventBus& operator=(EventBus&&) noexcept;

    void publish(const MidiPacket& packet);

private:
    struct Impl;
    std::string events_endpoint_;
    std::unique_ptr<Impl> impl_;
};

}  // namespace raptor::midi_io
