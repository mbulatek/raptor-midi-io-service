#include "raptor_midi_io/gpio_manager.hpp"

#include <memory>
#include <spdlog/spdlog.h>
#include <utility>
#include <vector>

#if defined(__linux__) && RAPTOR_MIDI_IO_HAS_GPIOD
#include <gpiod.h>
#endif

namespace raptor::midi_io {
namespace {

#if defined(__linux__) && RAPTOR_MIDI_IO_HAS_GPIOD

bool is_active(enum gpiod_line_value value) {
    return value == GPIOD_LINE_VALUE_ACTIVE;
}

struct EdgeEventBufferDeleter {
    void operator()(gpiod_edge_event_buffer* buffer) const {
        if (buffer != nullptr) {
            gpiod_edge_event_buffer_free(buffer);
        }
    }
};

#endif

}  // namespace

struct GpioManager::Impl {
    struct Watch {
        ModuleConfig module;
        bool last_active {false};
#if defined(__linux__) && RAPTOR_MIDI_IO_HAS_GPIOD
        gpiod_chip* chip {nullptr};
        gpiod_line_request* request {nullptr};
        unsigned int offset {0};
#endif
    };

    std::vector<Watch> watches;
#if defined(__linux__) && RAPTOR_MIDI_IO_HAS_GPIOD
    std::unique_ptr<gpiod_edge_event_buffer, EdgeEventBufferDeleter> event_buffer {
        gpiod_edge_event_buffer_new(16)
    };
#endif
};

GpioManager::GpioManager() : impl_(std::make_unique<Impl>()) {}

GpioManager::~GpioManager() {
#if defined(__linux__) && RAPTOR_MIDI_IO_HAS_GPIOD
    if (impl_) {
        for (auto& watch : impl_->watches) {
            if (watch.request != nullptr) {
                gpiod_line_request_release(watch.request);
            }
            if (watch.chip != nullptr) {
                gpiod_chip_close(watch.chip);
            }
        }
    }
#endif
}

GpioManager::GpioManager(GpioManager&&) noexcept = default;
GpioManager& GpioManager::operator=(GpioManager&&) noexcept = default;

void GpioManager::register_module(const ModuleConfig& module, HandshakeCallback callback) {
    callback_ = std::move(callback);

    Impl::Watch watch {.module = module};

#if defined(__linux__) && RAPTOR_MIDI_IO_HAS_GPIOD
    if (module.handshake_gpio >= 0) {
        watch.offset = static_cast<unsigned int>(module.handshake_gpio);
        watch.chip = gpiod_chip_open("/dev/gpiochip0");
        if (watch.chip == nullptr) {
            spdlog::warn("failed to open /dev/gpiochip0 for handshake GPIO {}", module.handshake_gpio);
        } else {
            gpiod_line_settings* settings = gpiod_line_settings_new();
            gpiod_line_config* line_config = gpiod_line_config_new();
            gpiod_request_config* request_config = gpiod_request_config_new();

            if (settings == nullptr || line_config == nullptr || request_config == nullptr) {
                spdlog::warn("failed to allocate libgpiod objects for handshake GPIO {}", module.handshake_gpio);
                gpiod_line_settings_free(settings);
                gpiod_line_config_free(line_config);
                gpiod_request_config_free(request_config);
            } else {
                const unsigned int offsets[] = {watch.offset};
                gpiod_request_config_set_consumer(request_config, "raptor-midi-io-service");

                const bool configured =
                    gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_INPUT) == 0 &&
                    gpiod_line_settings_set_edge_detection(settings, GPIOD_LINE_EDGE_BOTH) == 0;
                gpiod_line_settings_set_active_low(settings, module.handshake_active_low);
                const bool line_settings_applied =
                    configured &&
                    gpiod_line_config_add_line_settings(line_config, offsets, 1, settings) == 0;

                if (!line_settings_applied) {
                    spdlog::warn("failed to configure handshake GPIO {}", module.handshake_gpio);
                } else {
                    watch.request = gpiod_chip_request_lines(watch.chip, request_config, line_config);
                    if (watch.request == nullptr) {
                        spdlog::warn("failed to request handshake GPIO {}", module.handshake_gpio);
                    } else {
                        const auto value = gpiod_line_request_get_value(watch.request, watch.offset);
                        watch.last_active = value != GPIOD_LINE_VALUE_ERROR && is_active(value);
                    }
                }

                gpiod_line_settings_free(settings);
                gpiod_line_config_free(line_config);
                gpiod_request_config_free(request_config);
            }
        }
    }
#endif

    impl_->watches.push_back(std::move(watch));
    spdlog::debug("gpio watch module={} handshake_gpio={} active_low={}", module.id, module.handshake_gpio, module.handshake_active_low);
}

void GpioManager::run_once() {
    if (!callback_) {
        return;
    }

#if defined(__linux__) && RAPTOR_MIDI_IO_HAS_GPIOD
    for (auto& watch : impl_->watches) {
        if (watch.request == nullptr || !impl_->event_buffer) {
            continue;
        }

        for (;;) {
            const auto wait_result = gpiod_line_request_wait_edge_events(watch.request, 0);
            if (wait_result <= 0) {
                break;
            }

            const auto count = gpiod_line_request_read_edge_events(watch.request, impl_->event_buffer.get(), 16);
            if (count < 0) {
                break;
            }

            for (int index = 0; index < count; ++index) {
                const auto* event = gpiod_edge_event_buffer_get_event(
                    impl_->event_buffer.get(), static_cast<unsigned long>(index));
                if (event == nullptr) {
                    continue;
                }

                const auto value = gpiod_line_request_get_value(watch.request, watch.offset);
                if (value == GPIOD_LINE_VALUE_ERROR) {
                    continue;
                }

                const bool active = is_active(value);
                if (active && !watch.last_active) {
                    spdlog::debug("handshake edge module={} gpio={}", watch.module.id, watch.module.handshake_gpio);
                    callback_(watch.module);
                }
                watch.last_active = active;
            }
        }
    }
#else
    for (const auto& watch : impl_->watches) {
        callback_(watch.module);
    }
    impl_->watches.clear();
#endif
}

}  // namespace raptor::midi_io
