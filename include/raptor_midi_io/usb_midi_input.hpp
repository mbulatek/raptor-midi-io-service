#pragma once

#include "raptor_midi_io/config.hpp"
#include "raptor_midi_io/event_bus.hpp"

#include <atomic>
#include <functional>
#include <memory>

namespace raptor::midi_io {

class UsbMidiInput {
public:
    using PacketCallback = std::function<void(MidiPacket)>;

    UsbMidiInput(const std::vector<UsbMidiControllerConfig>& controllers,
                 std::atomic<std::uint64_t>& sequence_counter,
                 PacketCallback callback);
    ~UsbMidiInput();

    UsbMidiInput(const UsbMidiInput&) = delete;
    UsbMidiInput& operator=(const UsbMidiInput&) = delete;
    UsbMidiInput(UsbMidiInput&&) noexcept;
    UsbMidiInput& operator=(UsbMidiInput&&) noexcept;

    void start();
    void stop();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace raptor::midi_io
