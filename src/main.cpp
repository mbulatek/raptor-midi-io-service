#include "raptor_midi_io/application.hpp"
#include "raptor_midi_io/config.hpp"
#include "raptor_midi_io/logger.hpp"

#include <exception>
#include <string>

#include <spdlog/spdlog.h>

int main(int argc, char** argv) {
    const std::string config_path = argc > 1 ? argv[1] : "/etc/raptor-midi-io-service/raptor-midi-io-service.yaml";

    try {
        auto config = raptor::midi_io::load_config(config_path);
        raptor::midi_io::initialize_logging("raptor-midi-io-service", config.logging.level);
        spdlog::info("loading config from {}", config_path);
        raptor::midi_io::Application app {std::move(config)};
        return app.run();
    } catch (const std::exception& ex) {
        raptor::midi_io::initialize_logging("raptor-midi-io-service", "info");
        spdlog::critical("fatal: {}", ex.what());
        return 1;
    }
}