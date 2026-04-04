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
    std::string control_endpoint;
    std::string service_name {"raptor-midi-io-service"};
    IoMetrics io_metrics;
};

class ControlServer {
public:
    using ReloadHandler = std::function<bool(std::string& error)>;

    ControlServer(std::string control_endpoint, const ServiceConfig& config, ReloadHandler reload_handler = {});
    ~ControlServer();

    ControlServer(const ControlServer&) = delete;
    ControlServer& operator=(const ControlServer&) = delete;
    ControlServer(ControlServer&&) noexcept;
    ControlServer& operator=(ControlServer&&) noexcept;

    void set_snapshot(ServiceSnapshot snapshot);
    void poll_once();

private:
    struct Impl;

    std::string control_endpoint_;
    ReloadHandler reload_handler_ {};
    const ServiceConfig* config_ {nullptr};
    ServiceSnapshot snapshot_;
    std::unique_ptr<Impl> impl_;
};

}  // namespace raptor::midi_io
