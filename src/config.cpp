#include "raptor_midi_io/config.hpp"

#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <string>

namespace raptor::midi_io {
namespace {

template <typename T>
void assign_if_present(const YAML::Node& node, const char* key, T& target) {
    if (node[key]) {
        target = node[key].as<T>();
    }
}

void load_global_ipc_overrides(const std::string& path, IpcConfig& ipc) {
    if (path.empty()) {
        return;
    }

    const YAML::Node root = YAML::LoadFile(path);
    if (const auto ipc_root = root["ipc"]) {
        if (const auto midi_io = ipc_root["midi_io"]) {
            assign_if_present(midi_io, "events_endpoint", ipc.events_endpoint);
            assign_if_present(midi_io, "realtime_events_endpoint", ipc.realtime_events_endpoint);
            assign_if_present(midi_io, "playback_endpoint", ipc.playback_endpoint);
            assign_if_present(midi_io, "control_endpoint", ipc.control_endpoint);
        }
    }
}

void validate_config(const ServiceConfig& config) {
    if (config.modules.empty() && config.usb_midi_controllers.empty()) {
        throw std::runtime_error("config must define at least one SPI module or one usb_midi_controller");
    }
    if (config.queue.max_events_per_bus == 0) {
        throw std::runtime_error("queue.max_events_per_bus must be greater than zero");
    }
    if (config.queue.warning_threshold_percent > 100) {
        throw std::runtime_error("queue.warning_threshold_percent must be between 0 and 100");
    }
    if (config.queue.overflow_policy != "drop_oldest") {
        throw std::runtime_error("unsupported overflow policy: " + config.queue.overflow_policy);
    }
    if (config.logging.level.empty()) {
        throw std::runtime_error("logging.level must not be empty");
    }

    for (const auto& module : config.modules) {
        if (module.id.empty()) {
            throw std::runtime_error("each module must define id");
        }
        if (module.spi_device.empty()) {
            throw std::runtime_error("module " + module.id + " must define spi_device");
        }
        if (module.handshake_gpio < 0) {
            throw std::runtime_error("module " + module.id + " must define handshake_gpio");
        }
        if (module.chip_select_gpio < -1) {
            throw std::runtime_error(
                "module " + module.id + " must define chip_select_gpio >= -1 (-1 means hardware CS)");
        }
        if (module.midi_port_count < 1 || module.midi_port_count > 3) {
            throw std::runtime_error("module " + module.id + " must define midi_port_count between 1 and 3");
        }
        if (module.tx_interframe_delay_us > 50'000) {
            throw std::runtime_error("module " + module.id + " defines tx_interframe_delay_us > 50000");
        }
    }

    for (const auto& controller : config.usb_midi_controllers) {
        if (!controller.enabled) {
            continue;
        }
        if (controller.id.empty()) {
            throw std::runtime_error("each enabled usb_midi_controller must define id");
        }
        if (controller.match_name.empty()) {
            throw std::runtime_error("usb_midi_controller " + controller.id + " must define match_name");
        }
    }
}

void assign_global_port_ranges(ServiceConfig& config) {
    std::size_t next_global_port = 1;

    for (auto& module : config.modules) {
        module.first_global_midi_port = next_global_port;
        module.last_global_midi_port = next_global_port + module.midi_port_count - 1;
        next_global_port = module.last_global_midi_port + 1;
    }
}

}  // namespace

ServiceConfig load_config(const std::string& path) {
    if (path.empty()) {
        throw std::invalid_argument("config path must not be empty");
    }

    ServiceConfig config;
    config.config_path = path;

    const YAML::Node root = YAML::LoadFile(path);
    assign_if_present(root, "ipc_config_path", config.ipc_config_path);

    load_global_ipc_overrides(config.ipc_config_path, config.ipc);

    if (const auto logging = root["logging"]) {
        assign_if_present(logging, "level", config.logging.level);
    }

    if (const auto queue = root["queue"]) {
        assign_if_present(queue, "max_events_per_bus", config.queue.max_events_per_bus);
        assign_if_present(queue, "warning_threshold_percent", config.queue.warning_threshold_percent);
        assign_if_present(queue, "overflow_policy", config.queue.overflow_policy);
    }

    if (const auto modules = root["modules"]) {
        if (!modules.IsSequence()) {
            throw std::runtime_error("modules must be a YAML sequence");
        }

        for (const auto& node : modules) {
            ModuleConfig module;
            assign_if_present(node, "id", module.id);
            assign_if_present(node, "spi_device", module.spi_device);
            assign_if_present(node, "spi_speed_hz", module.spi_speed_hz);
            assign_if_present(node, "spi_mode", module.spi_mode);
            assign_if_present(node, "chip_select_gpio", module.chip_select_gpio);
            assign_if_present(node, "handshake_gpio", module.handshake_gpio);
            assign_if_present(node, "handshake_active_low", module.handshake_active_low);
            assign_if_present(node, "max_frame_bytes", module.max_frame_bytes);
            assign_if_present(node, "tx_interframe_delay_us", module.tx_interframe_delay_us);
            assign_if_present(node, "midi_port_count", module.midi_port_count);
            config.modules.push_back(std::move(module));
        }
    }

    if (const auto controllers = root["usb_midi_controllers"]) {
        if (!controllers.IsSequence()) {
            throw std::runtime_error("usb_midi_controllers must be a YAML sequence");
        }

        for (const auto& node : controllers) {
            UsbMidiControllerConfig controller;
            assign_if_present(node, "id", controller.id);
            assign_if_present(node, "match_name", controller.match_name);
            assign_if_present(node, "enabled", controller.enabled);
            config.usb_midi_controllers.push_back(std::move(controller));
        }
    }

    assign_global_port_ranges(config);
    validate_config(config);
    return config;
}

}  // namespace raptor::midi_io
