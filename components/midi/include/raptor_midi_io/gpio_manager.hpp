#pragma once

#include "raptor_midi_io/config.hpp"

#include <functional>
#include <memory>
#include <vector>

namespace raptor::midi_io {

class GpioManager {
public:
    using HandshakeCallback = std::function<void(const ModuleConfig&)>;

    GpioManager();
    ~GpioManager();

    GpioManager(const GpioManager&) = delete;
    GpioManager& operator=(const GpioManager&) = delete;
    GpioManager(GpioManager&&) noexcept;
    GpioManager& operator=(GpioManager&&) noexcept;

    void register_module(const ModuleConfig& module, HandshakeCallback callback);
    void run_once();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    HandshakeCallback callback_;
};

}  // namespace raptor::midi_io
