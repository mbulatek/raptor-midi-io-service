#include "raptor_midi_io/control_server.hpp"

#include <chrono>
#include <filesystem>
#include <string>

#include <spdlog/spdlog.h>
#include <string_view>
#include <utility>

#include <nlohmann/json.hpp>

#if RAPTOR_MIDI_IO_HAS_ZEROMQ
#include <zmq.h>
#endif

namespace raptor::midi_io {
namespace {

using json = nlohmann::json;

constexpr char kSchemaVersion[] = "1.0";

std::filesystem::path endpoint_directory(const std::string& endpoint) {
    constexpr std::string_view prefix {"ipc://"};
    if (!endpoint.starts_with(prefix)) {
        return {};
    }

    auto path = endpoint.substr(prefix.size());
    return std::filesystem::path {path}.parent_path();
}

std::uint64_t monotonic_time_ns() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}

json render_io_metrics(const IoMetrics& metrics) {
    json buses = json::array();
    for (const auto& bus : metrics.buses) {
        buses.push_back({
            {"bus_key", bus.bus_key},
            {"module_count", bus.module_count},
            {"queue_depth", bus.queue_depth},
            {"queue_capacity", bus.queue_capacity},
            {"queue_high_watermark", bus.queue_high_watermark},
            {"dropped_events", bus.dropped_events},
        });
    }

    return {
        {"dropped_events_total", metrics.dropped_events_total},
        {"queue_capacity_per_bus", metrics.queue_capacity_per_bus},
        {"warning_threshold_percent", metrics.warning_threshold_percent},
        {"buses", std::move(buses)},
    };
}

json render_module(const ModuleConfig& module) {
    return {
        {"id", module.id},
        {"spi_device", module.spi_device},
        {"spi_speed_hz", module.spi_speed_hz},
        {"spi_mode", module.spi_mode},
        {"chip_select_gpio", module.chip_select_gpio},
        {"handshake_gpio", module.handshake_gpio},
        {"handshake_active_low", module.handshake_active_low},
        {"max_frame_bytes", module.max_frame_bytes},
        {"midi_port_count", module.midi_port_count},
        {"first_global_midi_port", module.first_global_midi_port},
        {"last_global_midi_port", module.last_global_midi_port},
    };
}

struct ControlRequest {
    std::string command;
    std::string request_id;
    bool parse_error {false};
};

ControlRequest parse_request(const std::string& request_text) {
    try {
        const auto root = json::parse(request_text);
        ControlRequest request;

        if (!root.is_object()) {
            request.parse_error = true;
            return request;
        }

        if (root.contains("command") && root["command"].is_string()) {
            request.command = root["command"].get<std::string>();
        }
        if (root.contains("request_id") && root["request_id"].is_string()) {
            request.request_id = root["request_id"].get<std::string>();
        }
        return request;
    } catch (const std::exception&) {
        return ControlRequest {.parse_error = true};
    }
}

json envelope(const std::string& command, const std::string& request_id, const ServiceSnapshot& snapshot, bool ok) {
    json root = {
        {"schema_version", kSchemaVersion},
        {"service", snapshot.service_name},
        {"timestamp_ns", monotonic_time_ns()},
        {"ok", ok},
        {"command", command},
        {"request_id", request_id.empty() ? json(nullptr) : json(request_id)},
    };
    return root;
}

std::string ok_reply(const std::string& command,
                     const std::string& request_id,
                     const ServiceSnapshot& snapshot,
                     json data) {
    auto root = envelope(command, request_id, snapshot, true);
    root["data"] = std::move(data);
    return root.dump();
}

std::string error_reply(const std::string& command,
                        const std::string& request_id,
                        const ServiceSnapshot& snapshot,
                        const std::string& error_code,
                        const std::string& message) {
    auto root = envelope(command, request_id, snapshot, false);
    root["error"] = {
        {"code", error_code},
        {"message", message},
    };
    return root.dump();
}

std::string render_reply(const ControlRequest& request,
                         const ServiceConfig& config,
                         const ServiceSnapshot& snapshot) {
    const auto& command = request.command;
    if (command == "help") {
        return ok_reply(command, request.request_id, snapshot,
                        json {{"commands", json::array({"help", "ping", "status", "list-modules", "reload-config"})}});
    }
    if (command == "ping") {
        return ok_reply(command, request.request_id, snapshot, json {{"message", "pong"}});
    }
    if (command == "status") {
        json modules = json::array();
        std::size_t midi_port_total = 0;
        for (const auto& module : config.modules) {
            modules.push_back(render_module(module));
            midi_port_total += module.midi_port_count;
        }

        return ok_reply(command, request.request_id, snapshot,
                        json {
                            {"module_count", snapshot.module_count},
                            {"midi_port_total", midi_port_total},
                            {"published_packets", snapshot.published_packets},
                            {"events_endpoint", snapshot.events_endpoint},
                            {"control_endpoint", snapshot.control_endpoint},
                            {"service_name", snapshot.service_name},
                            {"modules", std::move(modules)},
                            {"io_metrics", render_io_metrics(snapshot.io_metrics)},
                        });
    }
    if (command == "list-modules") {
        json modules = json::array();
        for (const auto& module : config.modules) {
            modules.push_back(render_module(module));
        }
        return ok_reply(command, request.request_id, snapshot, json {{"modules", std::move(modules)}});
    }
    if (request.parse_error) {
        return error_reply("unknown", request.request_id, snapshot, "invalid-json",
                           "Request body is not valid JSON");
    }
    if (command.empty()) {
        return error_reply("unknown", request.request_id, snapshot, "invalid-request",
                           "Request must contain a string field named command");
    }
    return error_reply(command, request.request_id, snapshot, "unknown-command",
                       "Command is not supported");
}

}  // namespace

struct ControlServer::Impl {
#if RAPTOR_MIDI_IO_HAS_ZEROMQ
    void* context {nullptr};
    void* router {nullptr};
#endif
};

ControlServer::ControlServer(std::string control_endpoint, const ServiceConfig& config, ReloadHandler reload_handler)
    : control_endpoint_(std::move(control_endpoint)), reload_handler_(std::move(reload_handler)), config_(&config), impl_(std::make_unique<Impl>()) {
    snapshot_.module_count = config.modules.size();
    snapshot_.events_endpoint = config.ipc.events_endpoint;
    snapshot_.control_endpoint = config.ipc.control_endpoint;

#if RAPTOR_MIDI_IO_HAS_ZEROMQ
    const auto directory = endpoint_directory(control_endpoint_);
    if (!directory.empty()) {
        std::error_code ec;
        std::filesystem::create_directories(directory, ec);
        if (ec) {
            spdlog::warn("failed to create ZeroMQ control directory {}: {}", directory.string(), ec.message());
        }
    }

    impl_->context = zmq_ctx_new();
    if (impl_->context == nullptr) {
        spdlog::error("zmq_ctx_new failed for control endpoint {}: {}", control_endpoint_, zmq_strerror(zmq_errno()));
        return;
    }

    impl_->router = zmq_socket(impl_->context, ZMQ_ROUTER);
    if (impl_->router == nullptr) {
        spdlog::error("zmq_socket(ZMQ_ROUTER) failed for control endpoint {}: {}", control_endpoint_, zmq_strerror(zmq_errno()));
        zmq_ctx_term(impl_->context);
        impl_->context = nullptr;
        return;
    }

    constexpr int linger_ms = 0;
    (void)zmq_setsockopt(impl_->router, ZMQ_LINGER, &linger_ms, sizeof(linger_ms));

    if (zmq_bind(impl_->router, control_endpoint_.c_str()) != 0) {
        spdlog::error("zmq_bind failed for control endpoint {}: {}", control_endpoint_, zmq_strerror(zmq_errno()));
        zmq_close(impl_->router);
        zmq_ctx_term(impl_->context);
        impl_->router = nullptr;
        impl_->context = nullptr;
    }
#endif
}

ControlServer::~ControlServer() {
#if RAPTOR_MIDI_IO_HAS_ZEROMQ
    if (impl_) {
        if (impl_->router != nullptr) {
            zmq_close(impl_->router);
        }
        if (impl_->context != nullptr) {
            zmq_ctx_term(impl_->context);
        }
    }
#endif
}

ControlServer::ControlServer(ControlServer&&) noexcept = default;
ControlServer& ControlServer::operator=(ControlServer&&) noexcept = default;

void ControlServer::set_snapshot(ServiceSnapshot snapshot) {
    snapshot_ = std::move(snapshot);
}

void ControlServer::poll_once() {
#if RAPTOR_MIDI_IO_HAS_ZEROMQ
    if (!(impl_ && impl_->router != nullptr && config_ != nullptr)) {
        return;
    }

    zmq_pollitem_t items[] = {{impl_->router, 0, ZMQ_POLLIN, 0}};
    if (zmq_poll(items, 1, 0) <= 0 || (items[0].revents & ZMQ_POLLIN) == 0) {
        return;
    }

    zmq_msg_t identity;
    zmq_msg_init(&identity);
    if (zmq_msg_recv(&identity, impl_->router, 0) < 0) {
        zmq_msg_close(&identity);
        return;
    }

    zmq_msg_t command_msg;
    zmq_msg_init(&command_msg);
    if (zmq_msg_recv(&command_msg, impl_->router, 0) < 0) {
        zmq_msg_close(&identity);
        zmq_msg_close(&command_msg);
        return;
    }

    const auto* data = static_cast<const char*>(zmq_msg_data(&command_msg));
    const std::string request_text {data, data + zmq_msg_size(&command_msg)};
    const auto request = parse_request(request_text);
    spdlog::debug("control req endpoint={} bytes={} command={} request_id={} parse_error={}", control_endpoint_, request_text.size(), request.command, request.request_id, request.parse_error);

    std::string reply;
    if (request.command == "reload-config") {
        if (!reload_handler_) {
            reply = error_reply(
                request.command,
                request.request_id,
                snapshot_,
                "unsupported",
                "reload-config is not configured");
        } else {
            std::string error;
            const bool ok = reload_handler_(error);
            if (!ok) {
                reply = error_reply(
                    request.command,
                    request.request_id,
                    snapshot_,
                    "reload-failed",
                    error.empty() ? "reload failed" : error);
            } else {
                // Snapshot will be refreshed by the main loop; here we just acknowledge.
                json data = {
                    {"message", "reloaded"},
                    {"config_path", config_ ? config_->config_path : std::string{}},
                    {"module_count", config_ ? config_->modules.size() : 0},
                    {"usb_midi_controller_count", config_ ? config_->usb_midi_controllers.size() : 0}
                };
                reply = ok_reply(request.command, request.request_id, snapshot_, std::move(data));
            }
        }
    } else {
        reply = render_reply(request, *config_, snapshot_);
    }
    spdlog::debug("control reply endpoint={} bytes={} command={} ok_hint={}", control_endpoint_, reply.size(), request.command, request.parse_error ? "parse-error" : "rendered");

    (void)zmq_send(impl_->router, zmq_msg_data(&identity), zmq_msg_size(&identity), ZMQ_SNDMORE);
    (void)zmq_send(impl_->router, reply.data(), reply.size(), 0);

    zmq_msg_close(&identity);
    zmq_msg_close(&command_msg);
#endif
}

}  // namespace raptor::midi_io
