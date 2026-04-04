#pragma once

#include "raptor_midi_io/config.hpp"

#include <atomic>

namespace raptor::midi_io {

class Application {
public:
    explicit Application(ServiceConfig config);
    int run();

private:
    ServiceConfig config_;
    std::atomic<std::uint64_t> published_packets_ {0};
    std::atomic<std::uint64_t> sequence_counter_ {0};
};

}  // namespace raptor::midi_io
