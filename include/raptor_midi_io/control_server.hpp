#pragma once

#include "raptor_midi_io/config.hpp"
#include "raptor_midi_io/io_loop.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace raptor::midi_io {

struct ServiceSnapshot {
    std::size_t module_count {0};
    std::uint64_t published_packets {0};
    std::string events_endpoint;
    std::string realtime_events_endpoint;
    std::string playback_endpoint;
    std::string control_endpoint;
    std::string service_name {"raptor-midi-io-service"};
    IoMetrics io_metrics;
};

class ControlServer {
public:
    using ReloadHandler = std::function<bool(std::string& error)>;
    using SendMidiHandler = std::function<bool(std::size_t global_port, const std::vector<std::uint8_t>& bytes, std::string& error)>;
    using UpsertRouteHandler = std::function<bool(const MidiRouteConfig& route, std::string& error)>;
    using RemoveRouteHandler = std::function<bool(const std::string& route_id, std::string& error)>;
    using SetActiveRouteHandler = std::function<bool(const std::string& route_id, std::string& error)>;

    ControlServer(
        std::string control_endpoint,
        const ServiceConfig& config,
        ReloadHandler reload_handler = {},
        SendMidiHandler send_midi_handler = {},
        UpsertRouteHandler upsert_route_handler = {},
        RemoveRouteHandler remove_route_handler = {},
        SetActiveRouteHandler set_active_route_handler = {});
    ~ControlServer();

    ControlServer(const ControlServer&) = delete;
    ControlServer& operator=(const ControlServer&) = delete;
    ControlServer(ControlServer&&) noexcept;
    ControlServer& operator=(ControlServer&&) noexcept;

    void set_snapshot(ServiceSnapshot snapshot);
    // Returns true when a request was handled, false when no request was pending.
    bool poll_once();

private:
    struct Impl;

    std::string control_endpoint_;
    ReloadHandler reload_handler_ {};
    SendMidiHandler send_midi_handler_ {};
    UpsertRouteHandler upsert_route_handler_ {};
    RemoveRouteHandler remove_route_handler_ {};
    SetActiveRouteHandler set_active_route_handler_ {};
    const ServiceConfig* config_ {nullptr};
    ServiceSnapshot snapshot_;
    std::unique_ptr<Impl> impl_;
};

}  // namespace raptor::midi_io
