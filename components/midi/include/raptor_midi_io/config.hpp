#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace raptor::midi_io {

struct ModuleConfig {
    std::string id;
    std::string spi_device;
    std::uint32_t spi_speed_hz {1'000'000};
    std::uint8_t spi_mode {0};
    int chip_select_gpio {-1};
    int handshake_gpio {-1};
    bool handshake_active_low {false};
    std::size_t max_frame_bytes {64};
    std::uint32_t tx_interframe_delay_us {250};
    std::size_t midi_port_count {1};
    std::size_t first_global_midi_port {1};
    std::size_t last_global_midi_port {1};
};

struct UsbMidiControllerConfig {
    std::string id;
    std::string match_name;
    bool enabled {false};
};

struct IpcConfig {
    std::string events_endpoint {"ipc:///run/raptor-midi-io/events.zmq"};
    std::string realtime_events_endpoint {"ipc:///run/raptor-midi-io/events-rt.zmq"};
    std::string playback_endpoint {"ipc:///run/raptor-midi-io/playback-rt.zmq"};
    std::string control_endpoint {"ipc:///run/raptor-midi-io/control.zmq"};
};

struct QueueConfig {
    std::size_t max_events_per_bus {256};
    std::size_t warning_threshold_percent {75};
    std::string overflow_policy {"drop_oldest"};
};

struct LoggingConfig {
    std::string level {"info"};
};

struct ServiceConfig {
    std::string config_path;
    std::string ipc_config_path {"/etc/raptor/raptor-ipc.yaml"};
    std::vector<ModuleConfig> modules;
    std::vector<UsbMidiControllerConfig> usb_midi_controllers;
    IpcConfig ipc;
    QueueConfig queue;
    LoggingConfig logging;
};

ServiceConfig load_config(const std::string& path);

}  // namespace raptor::midi_io
