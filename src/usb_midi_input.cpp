#include "raptor_midi_io/usb_midi_input.hpp"

#include <alsa/asoundlib.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <memory>

#include <spdlog/spdlog.h>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <unordered_map>

namespace raptor::midi_io {
namespace {

using Clock = std::chrono::steady_clock;

struct ConnectedPort {
    UsbMidiControllerConfig config;
    int client_id {0};
    int port_id {0};
    std::string client_name;
    std::string port_name;
    std::size_t global_port {0};
};

bool contains_case_insensitive(const std::string& haystack, const std::string& needle) {
    if (needle.empty()) {
        return true;
    }

    auto it = std::search(
        haystack.begin(), haystack.end(), needle.begin(), needle.end(),
        [](char left, char right) {
            return std::tolower(static_cast<unsigned char>(left)) == std::tolower(static_cast<unsigned char>(right));
        });
    return it != haystack.end();
}

std::vector<std::uint8_t> midi_bytes_from_event(const snd_seq_event_t& event) {
    switch (event.type) {
    case SND_SEQ_EVENT_NOTEON:
        return {
            static_cast<std::uint8_t>(0x90 | ((event.data.note.channel - 1) & 0x0F)),
            static_cast<std::uint8_t>(event.data.note.note & 0x7F),
            static_cast<std::uint8_t>(event.data.note.velocity & 0x7F),
        };
    case SND_SEQ_EVENT_NOTEOFF:
        return {
            static_cast<std::uint8_t>(0x80 | ((event.data.note.channel - 1) & 0x0F)),
            static_cast<std::uint8_t>(event.data.note.note & 0x7F),
            static_cast<std::uint8_t>(event.data.note.velocity & 0x7F),
        };
    case SND_SEQ_EVENT_CONTROLLER:
        return {
            static_cast<std::uint8_t>(0xB0 | ((event.data.control.channel - 1) & 0x0F)),
            static_cast<std::uint8_t>(event.data.control.param & 0x7F),
            static_cast<std::uint8_t>(event.data.control.value & 0x7F),
        };
    default:
        return {};
    }
}

}  // namespace

struct UsbMidiInput::Impl {
    Impl(const std::vector<UsbMidiControllerConfig>& configured_controllers,
         std::size_t first_usb_global_port_in,
         std::atomic<std::uint64_t>& sequence,
         PacketCallback packet_callback)
        : controllers(configured_controllers),
          first_usb_global_port(first_usb_global_port_in),
          sequence_counter(sequence),
          callback(std::move(packet_callback)) {
        std::size_t next = first_usb_global_port;
        for (const auto& c : controllers) {
            if (!c.enabled) {
                continue;
            }
            (void)global_port_by_controller_id.emplace(c.id, next);
            ++next;
        }
    }

    void open_seq() {
        if (seq != nullptr) {
            return;
        }

        const int open_rc = snd_seq_open(&seq, "default", SND_SEQ_OPEN_INPUT, SND_SEQ_NONBLOCK);
        if (open_rc < 0) {
            spdlog::error("ALSA sequencer open failed: {}", snd_strerror(open_rc));
            throw std::runtime_error("failed to open ALSA sequencer for USB MIDI input");
        }

        snd_seq_set_client_name(seq, "raptor-midi-io-service");
        spdlog::debug("ALSA sequencer opened for USB MIDI");
        input_port = snd_seq_create_simple_port(
            seq,
            "usb-midi-in",
            SND_SEQ_PORT_CAP_WRITE | SND_SEQ_PORT_CAP_SUBS_WRITE,
            SND_SEQ_PORT_TYPE_APPLICATION);

        if (input_port < 0) {
            spdlog::error("ALSA create_simple_port failed");
            snd_seq_close(seq);
            seq = nullptr;
            throw std::runtime_error("failed to create ALSA sequencer input port");
        }
    }

    void connect_matching_ports() {
        if (seq == nullptr) {
            return;
        }

        snd_seq_client_info_t* client_info = nullptr;
        snd_seq_port_info_t* port_info = nullptr;
        snd_seq_client_info_alloca(&client_info);
        snd_seq_port_info_alloca(&port_info);

        snd_seq_client_info_set_client(client_info, -1);
        while (snd_seq_query_next_client(seq, client_info) >= 0) {
            const int client = snd_seq_client_info_get_client(client_info);
            const std::string client_name = snd_seq_client_info_get_name(client_info);

            snd_seq_port_info_set_client(port_info, client);
            snd_seq_port_info_set_port(port_info, -1);
            while (snd_seq_query_next_port(seq, port_info) >= 0) {
                const unsigned int capability = snd_seq_port_info_get_capability(port_info);
                if ((capability & SND_SEQ_PORT_CAP_READ) == 0 || (capability & SND_SEQ_PORT_CAP_SUBS_READ) == 0) {
                    continue;
                }

                const std::string port_name = snd_seq_port_info_get_name(port_info);
                for (const UsbMidiControllerConfig& controller : controllers) {
                    if (!controller.enabled) {
                        continue;
                    }
                    const std::string full_name = client_name + " " + port_name;
                    if (!contains_case_insensitive(full_name, controller.match_name)) {
                        continue;
                    }

                    if (snd_seq_connect_from(seq, input_port, client, snd_seq_port_info_get_port(port_info)) < 0) {
                        continue;
                    }

                    const auto g_it = global_port_by_controller_id.find(controller.id);
                    const std::size_t global_port = (g_it == global_port_by_controller_id.end()) ? 0 : g_it->second;

                    spdlog::debug("USB MIDI connect controller={} match=\"\"{}\"\" port={} alsa=\"\"{}:{}\"\"", controller.id, controller.match_name, global_port, client_name, port_name);
                    connected_ports.push_back(ConnectedPort {
                        .config = controller,
                        .client_id = client,
                        .port_id = snd_seq_port_info_get_port(port_info),
                        .client_name = client_name,
                        .port_name = port_name,
                        .global_port = global_port,
                    });
                }
            }
        }
    }

    void worker_loop() {
        open_seq();
        connect_matching_ports();

        while (!stop_requested.load(std::memory_order_relaxed)) {
            snd_seq_event_t* event = nullptr;
            bool drained_any = false;
            while (snd_seq_event_input(seq, &event) >= 0) {
                drained_any = true;
                if (event == nullptr) {
                    continue;
                }

                const auto bytes = midi_bytes_from_event(*event);
                if (bytes.empty()) {
                    continue;
                }

                auto it = std::find_if(connected_ports.begin(), connected_ports.end(), [&](const ConnectedPort& port) {
                    return port.client_id == event->source.client && port.port_id == event->source.port;
                });
                if (it == connected_ports.end()) {
                    continue;
                }

                MidiPacket packet;
                packet.module_id = it->config.id;
                packet.source_kind = "usb-midi-controller";
                packet.controller_id = it->config.id;
                packet.device_name = it->client_name + ":" + it->port_name;
                packet.module_port_count = 1;
                packet.module_first_global_port = it->global_port;
                packet.module_last_global_port = it->global_port;
                packet.local_port = 1;
                packet.global_port = it->global_port;
                packet.timestamp_ns = static_cast<std::uint64_t>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                        Clock::now().time_since_epoch())
                        .count());
                packet.bytes = bytes;
                packet.sequence = sequence_counter.fetch_add(1, std::memory_order_relaxed) + 1;

                spdlog::debug("usb midi packet controller={} seq={} bytes={} status=0x{:02X}", packet.controller_id, packet.sequence, packet.bytes.size(), packet.bytes.empty() ? 0 : packet.bytes[0]);
                callback(std::move(packet));
            }

            if (drained_any) {
                std::this_thread::yield();
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    }

    void start() {
        if (worker.joinable() || controllers.empty()) {
            return;
        }
        stop_requested.store(false, std::memory_order_relaxed);
        worker = std::thread([this]() {
            try {
                worker_loop();
            } catch (const std::exception& ex) {
                spdlog::error("USB MIDI input worker stopped: {}", ex.what());
            }
        });
    }

    void stop() {
        stop_requested.store(true, std::memory_order_relaxed);
        if (worker.joinable()) {
            worker.join();
        }
        if (seq != nullptr) {
            snd_seq_close(seq);
            seq = nullptr;
            input_port = -1;
        }
        connected_ports.clear();
    }

    std::size_t first_usb_global_port {1};
    std::unordered_map<std::string, std::size_t> global_port_by_controller_id;
    std::vector<UsbMidiControllerConfig> controllers;
    std::vector<ConnectedPort> connected_ports;
    std::atomic<std::uint64_t>& sequence_counter;
    PacketCallback callback;
    std::atomic<bool> stop_requested {false};
    std::thread worker;
    snd_seq_t* seq {nullptr};
    int input_port {-1};
};

UsbMidiInput::UsbMidiInput(const std::vector<UsbMidiControllerConfig>& controllers,
                           std::size_t first_usb_global_port,
                           std::atomic<std::uint64_t>& sequence_counter,
                           PacketCallback callback)
    : impl_(std::make_unique<Impl>(controllers, first_usb_global_port, sequence_counter, std::move(callback))) {}

UsbMidiInput::~UsbMidiInput() {
    stop();
}

UsbMidiInput::UsbMidiInput(UsbMidiInput&&) noexcept = default;
UsbMidiInput& UsbMidiInput::operator=(UsbMidiInput&&) noexcept = default;

void UsbMidiInput::start() {
    if (impl_) {
        impl_->start();
    }
}

void UsbMidiInput::stop() {
    if (impl_) {
        impl_->stop();
    }
}

}  // namespace raptor::midi_io
