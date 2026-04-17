#pragma once

#include "raptor_midi_io/config.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace raptor::midi_io {

struct SpiReadResult {
    std::size_t local_port {0};
    std::vector<std::uint8_t> bytes;
};

class SpiBus {
public:
    SpiBus() = default;
    SpiReadResult read_packet(const ModuleConfig& module);
    bool write_packet(const ModuleConfig& module, std::size_t local_port, const std::vector<std::uint8_t>& bytes);
};

}  // namespace raptor::midi_io
