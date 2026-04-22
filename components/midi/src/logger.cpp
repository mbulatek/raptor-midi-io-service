#include "raptor_midi_io/logger.hpp"

#include <cstdlib>

#include <spdlog/sinks/stdout_sinks.h>
#include <spdlog/spdlog.h>

namespace raptor::midi_io {

void initialize_logging(const std::string& service_name, const std::string& configured_level) {
    auto logger = spdlog::get(service_name);
    if (!logger) {
        logger = spdlog::stderr_logger_mt(service_name);
    }

    logger->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%n] [%l] %v");
    logger->set_level(spdlog::level::from_str(configured_level));
    if (const char* env_level = std::getenv("RAPTOR_LOG_LEVEL")) {
        logger->set_level(spdlog::level::from_str(env_level));
    }

    spdlog::set_default_logger(logger);
    spdlog::flush_on(spdlog::level::warn);
}

}  // namespace raptor::midi_io