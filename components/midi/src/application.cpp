#include "raptor_midi_io/application.hpp"

#include "raptor_midi_io/config.hpp"

#include "raptor_midi_io/control_server.hpp"
#include "raptor_midi_io/io_loop.hpp"
#include "raptor_midi_io/logger.hpp"

#include <chrono>
#include <memory>
#include <thread>

#include <spdlog/spdlog.h>

namespace raptor::midi_io {

Application::Application(ServiceConfig config) : config_(std::move(config)) {}

int Application::run() {
    auto io_loop = std::make_unique<IoLoop>(config_, published_packets_, sequence_counter_);
    io_loop->start();

    auto reload_handler = [&](std::string& error) -> bool {
        try {
            const auto new_config = load_config(config_.config_path);
            if (new_config.ipc.control_endpoint != config_.ipc.control_endpoint) {
                error = "control_endpoint changed; restart required";
                return false;
            }

            spdlog::info("reload-config: reloading from {}", config_.config_path);
            io_loop->stop();

            config_ = new_config;
            initialize_logging("raptor-midi-io-service", config_.logging.level);

            io_loop = std::make_unique<IoLoop>(config_, published_packets_, sequence_counter_);
            io_loop->start();

            spdlog::info(
                "reload-config: ok modules={} usb_controllers={} events={} events_rt={} playback_rt={} control={} log_level={}",
                config_.modules.size(),
                config_.usb_midi_controllers.size(),
                config_.ipc.events_endpoint,
                config_.ipc.realtime_events_endpoint,
                config_.ipc.playback_endpoint,
                config_.ipc.control_endpoint,
                config_.logging.level);
            return true;
        } catch (const std::exception& ex) {
            error = ex.what();
            spdlog::error("reload-config failed: {}", error);
            return false;
        }
    };

    auto send_midi_handler = [&](std::size_t global_port, const std::vector<std::uint8_t>& bytes, std::string& error) -> bool {
        return io_loop->send_midi(global_port, bytes, error);
    };

    auto upsert_route_handler = [&](const MidiRouteConfig& route, std::string& error) -> bool {
        return io_loop->upsert_route(route, error);
    };
    auto remove_route_handler = [&](const std::string& route_id, std::string& error) -> bool {
        return io_loop->remove_route(route_id, error);
    };
    auto set_active_route_handler = [&](const std::string& route_id, std::string& error) -> bool {
        return io_loop->set_active_route(route_id, error);
    };

    ControlServer control {
        config_.ipc.control_endpoint,
        config_,
        reload_handler,
        send_midi_handler,
        upsert_route_handler,
        remove_route_handler,
        set_active_route_handler};

    spdlog::info(
        "started with {} SPI modules and {} configured USB MIDI controller slots",
        config_.modules.size(),
        config_.usb_midi_controllers.size());
    spdlog::info("I/O loop handles SPI/USB MIDI upstream, SPI MIDI downstream, and MIDI event publication");

    for (;;) {
        control.set_snapshot(ServiceSnapshot {
            .module_count = config_.modules.size(),
            .published_packets = published_packets_.load(std::memory_order_relaxed),
            .events_endpoint = config_.ipc.events_endpoint,
            .realtime_events_endpoint = config_.ipc.realtime_events_endpoint,
            .playback_endpoint = config_.ipc.playback_endpoint,
            .control_endpoint = config_.ipc.control_endpoint,
            .service_name = "raptor-midi-io-service",
            .io_metrics = io_loop->snapshot(),
        });

        // Drain a bounded burst of control messages each tick.
        // Keeping this bounded reduces SPI TX bursts that can overrun the slave-side re-arm window.
        for (int i = 0; i < 64; ++i) {
            if (!control.poll_once()) {
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

}  // namespace raptor::midi_io
