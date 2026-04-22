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
            {"output_queue_depth", bus.output_queue_depth},
            {"output_queue_capacity", bus.output_queue_capacity},
            {"output_queue_high_watermark", bus.output_queue_high_watermark},
            {"output_dropped_events", bus.output_dropped_events},
            {"output_sent_events", bus.output_sent_events},
            {"output_failed_events", bus.output_failed_events},
        });
    }

    return {
        {"dropped_events_total", metrics.dropped_events_total},
        {"output_dropped_events_total", metrics.output_dropped_events_total},
        {"output_sent_events_total", metrics.output_sent_events_total},
        {"output_failed_events_total", metrics.output_failed_events_total},
        {"route_forwarded_events_total", metrics.route_forwarded_events_total},
        {"route_dropped_events_total", metrics.route_dropped_events_total},
        {"queue_capacity_per_bus", metrics.queue_capacity_per_bus},
        {"warning_threshold_percent", metrics.warning_threshold_percent},
        {"route_count", metrics.route_count},
        {"active_route_id", metrics.active_route_id},
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
    json data {json::object()};
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
        if (root.contains("data")) {
            request.data = root["data"];
        }
        return request;
    } catch (const std::exception&) {
        return ControlRequest {.parse_error = true};
    }
}

bool parse_send_midi_data(const json& data, std::size_t& global_port, std::vector<std::uint8_t>& bytes, std::string& error) {
    if (!data.is_object()) {
        error = "send-midi requires object field data";
        return false;
    }

    if (!data.contains("global_port") || !data["global_port"].is_number_integer()) {
        error = "send-midi requires integer data.global_port";
        return false;
    }
    const auto gp = data["global_port"].get<long long>();
    if (gp < 1) {
        error = "data.global_port must be >= 1";
        return false;
    }
    global_port = static_cast<std::size_t>(gp);

    if (!data.contains("bytes") || !data["bytes"].is_array()) {
        error = "send-midi requires array data.bytes";
        return false;
    }

    const auto& arr = data["bytes"];
    if (arr.empty() || arr.size() > 3) {
        error = "data.bytes must have 1..3 items";
        return false;
    }

    bytes.clear();
    bytes.reserve(arr.size());
    for (const auto& item : arr) {
        if (!item.is_number_integer()) {
            error = "data.bytes items must be integers";
            return false;
        }
        const auto value = item.get<long long>();
        if (value < 0 || value > 255) {
            error = "data.bytes items must be in range 0..255";
            return false;
        }
        bytes.push_back(static_cast<std::uint8_t>(value));
    }
    return true;
}

bool parse_set_route_data(const json& data, MidiRouteConfig& route, std::string& error) {
    if (!data.is_object()) {
        error = "set-route requires object field data";
        return false;
    }
    if (!data.contains("id") || !data["id"].is_string()) {
        error = "set-route requires string data.id";
        return false;
    }

    route = MidiRouteConfig {};
    route.id = data["id"].get<std::string>();
    route.midi_in_port = data.contains("midi_in_port") && data["midi_in_port"].is_number_integer()
        ? data["midi_in_port"].get<int>()
        : -1;
    route.midi_in_channel = data.contains("midi_in_channel") && data["midi_in_channel"].is_number_integer()
        ? data["midi_in_channel"].get<int>()
        : 0;
    route.midi_out_port = data.contains("midi_out_port") && data["midi_out_port"].is_number_integer()
        ? data["midi_out_port"].get<int>()
        : -1;
    route.midi_out_channel = data.contains("midi_out_channel") && data["midi_out_channel"].is_number_integer()
        ? data["midi_out_channel"].get<int>()
        : 1;
    route.enabled = !(data.contains("enabled") && data["enabled"].is_boolean() && !data["enabled"].get<bool>());
    return true;
}

bool parse_route_id_data(const json& data, std::string& route_id, std::string& error, const std::string& command_name) {
    if (!data.is_object()) {
        error = command_name + " requires object field data";
        return false;
    }
    if (!data.contains("id") || !data["id"].is_string()) {
        error = command_name + " requires string data.id";
        return false;
    }
    route_id = data["id"].get<std::string>();
    return true;
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
                        json {{"commands", json::array({
                            "help",
                            "ping",
                            "status",
                            "list-modules",
                            "reload-config",
                            "send-midi",
                            "set-route",
                            "remove-route",
                            "set-active-route",
                        })}});
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
                            {"realtime_events_endpoint", snapshot.realtime_events_endpoint},
                            {"playback_endpoint", snapshot.playback_endpoint},
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

ControlServer::ControlServer(
    std::string control_endpoint,
    const ServiceConfig& config,
    ReloadHandler reload_handler,
    SendMidiHandler send_midi_handler,
    UpsertRouteHandler upsert_route_handler,
    RemoveRouteHandler remove_route_handler,
    SetActiveRouteHandler set_active_route_handler)
    : control_endpoint_(std::move(control_endpoint)),
      reload_handler_(std::move(reload_handler)),
      send_midi_handler_(std::move(send_midi_handler)),
      upsert_route_handler_(std::move(upsert_route_handler)),
      remove_route_handler_(std::move(remove_route_handler)),
      set_active_route_handler_(std::move(set_active_route_handler)),
      config_(&config),
      impl_(std::make_unique<Impl>()) {
    snapshot_.module_count = config.modules.size();
    snapshot_.events_endpoint = config.ipc.events_endpoint;
    snapshot_.realtime_events_endpoint = config.ipc.realtime_events_endpoint;
    snapshot_.playback_endpoint = config.ipc.playback_endpoint;
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

bool ControlServer::poll_once() {
#if RAPTOR_MIDI_IO_HAS_ZEROMQ
    if (!(impl_ && impl_->router != nullptr && config_ != nullptr)) {
        return false;
    }

    zmq_pollitem_t items[] = {{impl_->router, 0, ZMQ_POLLIN, 0}};
    if (zmq_poll(items, 1, 0) <= 0 || (items[0].revents & ZMQ_POLLIN) == 0) {
        return false;
    }

    zmq_msg_t identity;
    zmq_msg_init(&identity);
    if (zmq_msg_recv(&identity, impl_->router, 0) < 0) {
        zmq_msg_close(&identity);
        return false;
    }
    if (zmq_msg_more(&identity) == 0) {
        zmq_msg_close(&identity);
        return false;
    }

    zmq_msg_t command_msg;
    zmq_msg_init(&command_msg);
    if (zmq_msg_recv(&command_msg, impl_->router, 0) < 0) {
        zmq_msg_close(&identity);
        zmq_msg_close(&command_msg);
        return false;
    }

    // ROUTER can receive:
    // - REQ:    [identity][empty][payload]
    // - DEALER: [identity][payload]
    // Consume optional delimiter and keep parser aligned.
    if (zmq_msg_size(&command_msg) == 0 && zmq_msg_more(&command_msg) != 0) {
        zmq_msg_close(&command_msg);
        zmq_msg_init(&command_msg);
        if (zmq_msg_recv(&command_msg, impl_->router, 0) < 0) {
            zmq_msg_close(&identity);
            zmq_msg_close(&command_msg);
            return false;
        }
    }

    // Drain any extra frames to avoid desynchronizing the next request.
    while (zmq_msg_more(&command_msg) != 0) {
        zmq_msg_t junk;
        zmq_msg_init(&junk);
        if (zmq_msg_recv(&junk, impl_->router, 0) < 0) {
            zmq_msg_close(&junk);
            break;
        }
        zmq_msg_close(&junk);
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
    } else if (request.command == "send-midi") {
        if (!send_midi_handler_) {
            reply = error_reply(
                request.command,
                request.request_id,
                snapshot_,
                "unsupported",
                "send-midi is not configured");
        } else {
            std::size_t global_port = 0;
            std::vector<std::uint8_t> bytes;
            std::string parse_error_message;
            if (!parse_send_midi_data(request.data, global_port, bytes, parse_error_message)) {
                reply = error_reply(
                    request.command,
                    request.request_id,
                    snapshot_,
                    "invalid-request",
                    parse_error_message);
            } else {
                std::string send_error;
                const bool ok = send_midi_handler_(global_port, bytes, send_error);
                if (!ok) {
                    reply = error_reply(
                        request.command,
                        request.request_id,
                        snapshot_,
                        "send-failed",
                        send_error.empty() ? "send-midi failed" : send_error);
                } else {
                    reply = ok_reply(
                        request.command,
                        request.request_id,
                        snapshot_,
                        json {
                            {"message", "queued"},
                            {"global_port", global_port},
                            {"bytes", bytes},
                        });
                }
            }
        }
    } else if (request.command == "set-route") {
        if (!upsert_route_handler_) {
            reply = error_reply(
                request.command,
                request.request_id,
                snapshot_,
                "unsupported",
                "set-route is not configured");
        } else {
            MidiRouteConfig route;
            std::string parse_error_message;
            if (!parse_set_route_data(request.data, route, parse_error_message)) {
                reply = error_reply(
                    request.command,
                    request.request_id,
                    snapshot_,
                    "invalid-request",
                    parse_error_message);
            } else {
                std::string route_error;
                const bool ok = upsert_route_handler_(route, route_error);
                if (!ok) {
                    reply = error_reply(
                        request.command,
                        request.request_id,
                        snapshot_,
                        "route-failed",
                        route_error.empty() ? "set-route failed" : route_error);
                } else {
                    reply = ok_reply(
                        request.command,
                        request.request_id,
                        snapshot_,
                        json {
                            {"message", "route-upserted"},
                            {"id", route.id},
                            {"midi_in_port", route.midi_in_port},
                            {"midi_in_channel", route.midi_in_channel},
                            {"midi_out_port", route.midi_out_port},
                            {"midi_out_channel", route.midi_out_channel},
                            {"enabled", route.enabled},
                        });
                }
            }
        }
    } else if (request.command == "remove-route") {
        if (!remove_route_handler_) {
            reply = error_reply(
                request.command,
                request.request_id,
                snapshot_,
                "unsupported",
                "remove-route is not configured");
        } else {
            std::string route_id;
            std::string parse_error_message;
            if (!parse_route_id_data(request.data, route_id, parse_error_message, "remove-route")) {
                reply = error_reply(
                    request.command,
                    request.request_id,
                    snapshot_,
                    "invalid-request",
                    parse_error_message);
            } else {
                std::string route_error;
                const bool ok = remove_route_handler_(route_id, route_error);
                if (!ok) {
                    reply = error_reply(
                        request.command,
                        request.request_id,
                        snapshot_,
                        "route-failed",
                        route_error.empty() ? "remove-route failed" : route_error);
                } else {
                    reply = ok_reply(
                        request.command,
                        request.request_id,
                        snapshot_,
                        json {
                            {"message", "route-removed"},
                            {"id", route_id},
                        });
                }
            }
        }
    } else if (request.command == "set-active-route") {
        if (!set_active_route_handler_) {
            reply = error_reply(
                request.command,
                request.request_id,
                snapshot_,
                "unsupported",
                "set-active-route is not configured");
        } else {
            std::string route_id;
            std::string parse_error_message;
            if (!parse_route_id_data(request.data, route_id, parse_error_message, "set-active-route")) {
                reply = error_reply(
                    request.command,
                    request.request_id,
                    snapshot_,
                    "invalid-request",
                    parse_error_message);
            } else {
                std::string route_error;
                const bool ok = set_active_route_handler_(route_id, route_error);
                if (!ok) {
                    reply = error_reply(
                        request.command,
                        request.request_id,
                        snapshot_,
                        "route-failed",
                        route_error.empty() ? "set-active-route failed" : route_error);
                } else {
                    reply = ok_reply(
                        request.command,
                        request.request_id,
                        snapshot_,
                        json {
                            {"message", "active-route-updated"},
                            {"id", route_id},
                        });
                }
            }
        }
    } else {
        reply = render_reply(request, *config_, snapshot_);
    }
    spdlog::debug("control reply endpoint={} bytes={} command={} ok_hint={}", control_endpoint_, reply.size(), request.command, request.parse_error ? "parse-error" : "rendered");

    (void)zmq_send(impl_->router, zmq_msg_data(&identity), zmq_msg_size(&identity), ZMQ_SNDMORE);
    // Keep reply framing compatible with REQ and DEALER clients.
    (void)zmq_send(impl_->router, "", 0, ZMQ_SNDMORE);
    (void)zmq_send(impl_->router, reply.data(), reply.size(), 0);

    zmq_msg_close(&identity);
    zmq_msg_close(&command_msg);
    return true;
#endif
    return false;
}

}  // namespace raptor::midi_io
