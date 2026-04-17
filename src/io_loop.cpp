#include "raptor_midi_io/io_loop.hpp"

#include "raptor_midi_io/event_bus.hpp"
#include "raptor_midi_io/gpio_manager.hpp"
#include "raptor_midi_io/spi_bus.hpp"
#include "raptor_midi_io/usb_midi_input.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <spdlog/spdlog.h>

namespace raptor::midi_io {

namespace {

std::size_t first_usb_global_port(const ServiceConfig& cfg) {
    std::size_t next = 1;
    for (const auto& m : cfg.modules) {
        next = m.last_global_midi_port + 1;
    }
    return next;
}

}  // namespace


struct IoLoop::Impl {
    struct PublishedItem {
        MidiPacket packet;
    };

    struct OutboundItem {
        std::size_t module_index {0};
        std::size_t local_port {1};
        std::size_t global_port {0};
        std::vector<std::uint8_t> bytes;
        std::uint64_t sequence {0};
    };

    struct BusWorker {
        std::string bus_key;
        std::vector<ModuleConfig> modules;
        std::deque<PublishedItem> queue;
        std::deque<OutboundItem> output_queue;
        std::size_t queue_capacity {0};
        std::size_t warning_threshold {0};
        std::size_t queue_high_watermark {0};
        std::uint64_t dropped_events {0};
        std::size_t output_queue_high_watermark {0};
        std::uint64_t output_dropped_events {0};
        std::uint64_t output_sent_events {0};
        std::uint64_t output_failed_events {0};
        GpioManager gpio;
        SpiBus spi;
        std::thread worker;
    };

    explicit Impl(const ServiceConfig& service_config,
                  std::atomic<std::uint64_t>& published_count,
                  std::atomic<std::uint64_t>& sequence_count)
        : config(service_config),
          published_packets(published_count),
          sequence_counter(sequence_count),
          usb_midi_input(config.usb_midi_controllers, first_usb_global_port(config), sequence_counter, [this](MidiPacket packet) {
              enqueue_locked(std::move(packet), usb_queue, usb_queue_high_watermark, usb_dropped_events);
              queue_cv.notify_one();
          }) {}

    void build_bus_workers() {
        std::unordered_map<std::string, std::size_t> index_by_bus;

        for (const auto& module : config.modules) {
            const auto it = index_by_bus.find(module.spi_device);
            if (it == index_by_bus.end()) {
                index_by_bus.emplace(module.spi_device, bus_workers.size());
                bus_workers.push_back(BusWorker {
                    .bus_key = module.spi_device,
                    .modules = {module},
                    .queue_capacity = config.queue.max_events_per_bus,
                    .warning_threshold =
                        (config.queue.max_events_per_bus * config.queue.warning_threshold_percent) / 100,
                });
            } else {
                bus_workers[it->second].modules.push_back(module);
            }
        }
    }

    void enqueue_locked(MidiPacket packet,
                        std::deque<PublishedItem>& queue,
                        std::size_t& high_watermark,
                        std::uint64_t& dropped_events) {
        std::lock_guard<std::mutex> lock(queue_mutex);
        if (config.queue.max_events_per_bus > 0 && queue.size() >= config.queue.max_events_per_bus) {
            queue.pop_front();
            ++dropped_events;
            if (dropped_events == 1 || (dropped_events % 100) == 0) {
                spdlog::warn(
                    "usb queue overflow dropped_events={} capacity={}",
                    dropped_events,
                    config.queue.max_events_per_bus);
            }
        }
        queue.push_back(PublishedItem {.packet = std::move(packet)});
        if (queue.size() > high_watermark) {
            high_watermark = queue.size();
        }
    }

    bool enqueue_output(std::size_t global_port, std::vector<std::uint8_t> bytes, std::string& error) {
        if (global_port == 0) {
            error = "global_port must be >= 1";
            return false;
        }
        if (bytes.empty() || bytes.size() > 3) {
            error = "bytes must have 1..3 MIDI bytes";
            return false;
        }
        if ((bytes[0] & 0x80) == 0) {
            error = "first byte must be a MIDI status byte (>= 0x80)";
            return false;
        }

        std::lock_guard<std::mutex> lock(queue_mutex);

        for (auto& worker : bus_workers) {
            for (std::size_t module_index = 0; module_index < worker.modules.size(); ++module_index) {
                const auto& module = worker.modules[module_index];
                if (global_port < module.first_global_midi_port || global_port > module.last_global_midi_port) {
                    continue;
                }

                const auto local_port = (global_port - module.first_global_midi_port) + 1;
                if (local_port < 1 || local_port > module.midi_port_count) {
                    error = "resolved local_port out of configured module range";
                    return false;
                }

                if (worker.queue_capacity > 0 && worker.output_queue.size() >= worker.queue_capacity) {
                    worker.output_queue.pop_front();
                    ++worker.output_dropped_events;
                    if (worker.output_dropped_events == 1 || (worker.output_dropped_events % 100) == 0) {
                        spdlog::warn(
                            "bus output queue overflow bus={} dropped_events={} capacity={}",
                            worker.bus_key,
                            worker.output_dropped_events,
                            worker.queue_capacity);
                    }
                }

                const auto sequence = sequence_counter.fetch_add(1, std::memory_order_relaxed) + 1;
                worker.output_queue.push_back(OutboundItem {
                    .module_index = module_index,
                    .local_port = local_port,
                    .global_port = global_port,
                    .bytes = std::move(bytes),
                    .sequence = sequence,
                });
                if (worker.output_queue.size() > worker.output_queue_high_watermark) {
                    worker.output_queue_high_watermark = worker.output_queue.size();
                }
                queue_cv.notify_one();
                return true;
            }
        }

        error = "no module route for global_port=" + std::to_string(global_port);
        return false;
    }

    void start_bus_worker(BusWorker& worker) {
        worker.worker = std::thread([this, &worker]() {
            for (const auto& module : worker.modules) {
                worker.gpio.register_module(module, [&](const ModuleConfig& triggered) {
                    auto read_result = worker.spi.read_packet(triggered);
                    if (read_result.bytes.empty()) {
                        return;
                    }

                    if (read_result.local_port < 1 || read_result.local_port > triggered.midi_port_count) {
                        spi_invalid_port_drops_total.fetch_add(1, std::memory_order_relaxed);
                        // Wake publisher loop so it can publish updated stats even if nothing was enqueued.
                        queue_cv.notify_one();
                        return;
                    }

                    const auto sequence = sequence_counter.fetch_add(1, std::memory_order_relaxed) + 1;
                    const auto global_port =
                        triggered.first_global_midi_port + read_result.local_port - 1;

                    MidiPacket packet {
                        .module_id = triggered.id,
                        .source_kind = "spi-module",
                        .controller_id = "",
                        .device_name = "",
                        .spi_device = triggered.spi_device,
                        .spi_speed_hz = triggered.spi_speed_hz,
                        .spi_mode = triggered.spi_mode,
                        .chip_select_gpio = triggered.chip_select_gpio,
                        .handshake_gpio = triggered.handshake_gpio,
                        .handshake_active_low = triggered.handshake_active_low,
                        .module_port_count = triggered.midi_port_count,
                        .module_first_global_port = triggered.first_global_midi_port,
                        .module_last_global_port = triggered.last_global_midi_port,
                        .local_port = read_result.local_port,
                        .global_port = global_port,
                        .bytes = std::move(read_result.bytes),
                        .sequence = sequence,
                    };

                    {
                        std::lock_guard<std::mutex> lock(queue_mutex);
                        if (worker.queue_capacity > 0 && worker.queue.size() >= worker.queue_capacity) {
                            worker.queue.pop_front();
                            ++worker.dropped_events;
                            if (worker.dropped_events == 1 || (worker.dropped_events % 100) == 0) {
                                spdlog::warn(
                                    "bus queue overflow bus={} dropped_events={} capacity={}",
                                    worker.bus_key,
                                    worker.dropped_events,
                                    worker.queue_capacity);
                            }
                        }

                        worker.queue.push_back(PublishedItem {.packet = std::move(packet)});
                        if (worker.queue.size() > worker.queue_high_watermark) {
                            worker.queue_high_watermark = worker.queue.size();
                        }
                    }
                    queue_cv.notify_one();
                });
            }

            for (;;) {
                OutboundItem outbound;
                bool have_outbound = false;
                {
                    std::lock_guard<std::mutex> lock(queue_mutex);
                    if (!worker.output_queue.empty()) {
                        outbound = std::move(worker.output_queue.front());
                        worker.output_queue.pop_front();
                        have_outbound = true;
                    }
                }

                if (have_outbound) {
                    bool sent = false;
                    if (outbound.module_index < worker.modules.size()) {
                        const auto& module = worker.modules[outbound.module_index];
                        sent = worker.spi.write_packet(module, outbound.local_port, outbound.bytes);
                        if (!sent) {
                            spdlog::warn(
                                "spi tx failed bus={} module={} global_port={} local_port={} bytes={} seq={}",
                                worker.bus_key,
                                module.id,
                                outbound.global_port,
                                outbound.local_port,
                                outbound.bytes.size(),
                                outbound.sequence);
                        } else {
                            spdlog::debug(
                                "spi tx bus={} module={} global_port={} local_port={} bytes={} seq={}",
                                worker.bus_key,
                                module.id,
                                outbound.global_port,
                                outbound.local_port,
                                outbound.bytes.size(),
                                outbound.sequence);
                        }
                    } else {
                        spdlog::warn(
                            "spi tx failed bus={} invalid module index={} seq={}",
                            worker.bus_key,
                            outbound.module_index,
                            outbound.sequence);
                    }

                    {
                        std::lock_guard<std::mutex> lock(queue_mutex);
                        if (sent) {
                            ++worker.output_sent_events;
                        } else {
                            ++worker.output_failed_events;
                        }
                    }
                    continue;
                }

                if (stop_requested.load(std::memory_order_relaxed)) {
                    break;
                }

                worker.gpio.run_once();
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        });
    }

    void publisher_loop() {
        EventBus bus {config.ipc.events_endpoint};
        spdlog::debug("io publisher start endpoint={}", config.ipc.events_endpoint);
        std::size_t next_bus_index = 0;

        MidiIoStats last_stats;
        bool last_stats_valid = false;
        auto last_stats_pub_at = std::chrono::steady_clock::now();

        for (;;) {
            PublishedItem item;
            bool found = false;
            bool should_stop = false;

            {
                std::unique_lock<std::mutex> lock(queue_mutex);
                (void)queue_cv.wait_for(lock, std::chrono::milliseconds(200), [&]() {
                    return stop_requested.load(std::memory_order_relaxed) || has_pending_items_locked();
                });

                should_stop = stop_requested.load(std::memory_order_relaxed) && !has_pending_items_locked();

                if (!usb_queue.empty()) {
                    item = std::move(usb_queue.front());
                    usb_queue.pop_front();
                    found = true;
                } else {
                    for (std::size_t i = 0; i < bus_workers.size(); ++i) {
                        const auto index = (next_bus_index + i) % bus_workers.size();
                        auto& worker = bus_workers[index];
                        if (worker.queue.empty()) {
                            continue;
                        }

                        item = std::move(worker.queue.front());
                        worker.queue.pop_front();
                        next_bus_index = (index + 1) % bus_workers.size();
                        found = true;
                        break;
                    }
                }
            }

            // Publish stats periodically or when they change.
            MidiIoStats stats;
            {
                std::lock_guard<std::mutex> lock(queue_mutex);
                stats.spi_invalid_port_drops_total = spi_invalid_port_drops_total.load(std::memory_order_relaxed);
                stats.usb_queue_dropped_events_total = usb_dropped_events;
                std::uint64_t bus_drops = 0;
                std::uint64_t out_drops = 0;
                std::uint64_t out_sent = 0;
                std::uint64_t out_failed = 0;
                for (const auto& w : bus_workers) {
                    bus_drops += w.dropped_events;
                    out_drops += w.output_dropped_events;
                    out_sent += w.output_sent_events;
                    out_failed += w.output_failed_events;
                }
                stats.bus_queue_dropped_events_total = bus_drops;
                stats.output_queue_dropped_events_total = out_drops;
                stats.output_sent_events_total = out_sent;
                stats.output_failed_events_total = out_failed;
            }

            const auto now = std::chrono::steady_clock::now();
            const bool changed =
                !last_stats_valid ||
                stats.spi_invalid_port_drops_total != last_stats.spi_invalid_port_drops_total ||
                stats.bus_queue_dropped_events_total != last_stats.bus_queue_dropped_events_total ||
                stats.usb_queue_dropped_events_total != last_stats.usb_queue_dropped_events_total ||
                stats.output_queue_dropped_events_total != last_stats.output_queue_dropped_events_total ||
                stats.output_sent_events_total != last_stats.output_sent_events_total ||
                stats.output_failed_events_total != last_stats.output_failed_events_total;

            if (!last_stats_valid || changed || (now - last_stats_pub_at) > std::chrono::seconds(1)) {
                bus.publish_stats(stats);
                last_stats = stats;
                last_stats_valid = true;
                last_stats_pub_at = now;
            }

            if (found) {
                bus.publish(item.packet);
                published_packets.fetch_add(1, std::memory_order_relaxed);
                continue;
            }

            if (should_stop) {
                break;
            }
        }

        spdlog::debug("io publisher stopped");
    }

    bool has_pending_items_locked() const {
        if (!usb_queue.empty()) {
            return true;
        }
        for (const auto& worker : bus_workers) {
            if (!worker.queue.empty()) {
                return true;
            }
        }
        return false;
    }

    void start() {
        if (!bus_workers.empty() || publisher.joinable()) {
            return;
        }

        build_bus_workers();
        spdlog::debug(
            "io start modules={} buses={} usb_controllers={} queue_capacity={} warn_threshold_percent={}",
            config.modules.size(),
            bus_workers.size(),
            config.usb_midi_controllers.size(),
            config.queue.max_events_per_bus,
            config.queue.warning_threshold_percent);
        for (const auto& w : bus_workers) {
            spdlog::debug(
                "io bus={} modules={} queue_capacity={} warn_threshold={}",
                w.bus_key,
                w.modules.size(),
                w.queue_capacity,
                w.warning_threshold);
            for (const auto& m : w.modules) {
                spdlog::debug(
                    "io module={} ports={} global_ports={}..{} spi={} speed_hz={} mode={} cs_gpio={} handshake_gpio={} active_low={}",
                    m.id,
                    m.midi_port_count,
                    m.first_global_midi_port,
                    m.last_global_midi_port,
                    m.spi_device,
                    m.spi_speed_hz,
                    m.spi_mode,
                    m.chip_select_gpio,
                    m.handshake_gpio,
                    m.handshake_active_low);
            }
        }
        stop_requested.store(false, std::memory_order_relaxed);

        publisher = std::thread([this]() { publisher_loop(); });
        usb_midi_input.start();
        for (auto& worker : bus_workers) {
            start_bus_worker(worker);
        }
    }

    void stop() {
        spdlog::debug("io stop requested");
        stop_requested.store(true, std::memory_order_relaxed);
        usb_midi_input.stop();
        queue_cv.notify_all();

        for (auto& worker : bus_workers) {
            if (worker.worker.joinable()) {
                worker.worker.join();
            }
        }
        if (publisher.joinable()) {
            publisher.join();
        }
        for (const auto& w : bus_workers) {
            if (w.dropped_events > 0 || w.queue_high_watermark > 0 ||
                w.output_dropped_events > 0 || w.output_queue_high_watermark > 0 ||
                w.output_sent_events > 0 || w.output_failed_events > 0) {
                spdlog::debug(
                    "io stop bus={} dropped_events={} high_watermark={} out_dropped={} out_high_watermark={} out_sent={} out_failed={}",
                    w.bus_key,
                    w.dropped_events,
                    w.queue_high_watermark,
                    w.output_dropped_events,
                    w.output_queue_high_watermark,
                    w.output_sent_events,
                    w.output_failed_events);
            }
        }
        if (usb_dropped_events > 0 || usb_queue_high_watermark > 0) {
            spdlog::debug(
                "io stop bus=usb-midi dropped_events={} high_watermark={}",
                usb_dropped_events,
                usb_queue_high_watermark);
        }
        bus_workers.clear();
        usb_queue.clear();
    }

    void enqueue(MidiPacket packet) {
        enqueue_locked(std::move(packet), usb_queue, usb_queue_high_watermark, usb_dropped_events);
        queue_cv.notify_one();
    }

    bool send_midi(std::size_t global_port, std::vector<std::uint8_t> bytes, std::string& error) {
        return enqueue_output(global_port, std::move(bytes), error);
    }

    IoMetrics snapshot() const {
        std::lock_guard<std::mutex> lock(queue_mutex);

        IoMetrics metrics;
        metrics.queue_capacity_per_bus = config.queue.max_events_per_bus;
        metrics.warning_threshold_percent = config.queue.warning_threshold_percent;
        metrics.dropped_events_total += usb_dropped_events;
        if (!config.usb_midi_controllers.empty()) {
            metrics.buses.push_back(BusIoMetrics {
                .bus_key = "usb-midi",
                .module_count = config.usb_midi_controllers.size(),
                .queue_depth = usb_queue.size(),
                .queue_capacity = config.queue.max_events_per_bus,
                .queue_high_watermark = usb_queue_high_watermark,
                .dropped_events = usb_dropped_events,
                .output_queue_depth = 0,
                .output_queue_capacity = config.queue.max_events_per_bus,
                .output_queue_high_watermark = 0,
                .output_dropped_events = 0,
                .output_sent_events = 0,
                .output_failed_events = 0,
            });
        }

        for (const auto& worker : bus_workers) {
            metrics.dropped_events_total += worker.dropped_events;
            metrics.output_dropped_events_total += worker.output_dropped_events;
            metrics.output_sent_events_total += worker.output_sent_events;
            metrics.output_failed_events_total += worker.output_failed_events;
            metrics.buses.push_back(BusIoMetrics {
                .bus_key = worker.bus_key,
                .module_count = worker.modules.size(),
                .queue_depth = worker.queue.size(),
                .queue_capacity = worker.queue_capacity,
                .queue_high_watermark = worker.queue_high_watermark,
                .dropped_events = worker.dropped_events,
                .output_queue_depth = worker.output_queue.size(),
                .output_queue_capacity = worker.queue_capacity,
                .output_queue_high_watermark = worker.output_queue_high_watermark,
                .output_dropped_events = worker.output_dropped_events,
                .output_sent_events = worker.output_sent_events,
                .output_failed_events = worker.output_failed_events,
            });
        }

        return metrics;
    }

    ServiceConfig config;
    std::atomic<std::uint64_t>& published_packets;
    std::atomic<std::uint64_t>& sequence_counter;
    std::atomic<std::uint64_t> spi_invalid_port_drops_total {0};

    UsbMidiInput usb_midi_input;
    std::atomic<bool> stop_requested {false};
    mutable std::mutex queue_mutex;
    std::condition_variable queue_cv;
    std::vector<BusWorker> bus_workers;
    std::deque<PublishedItem> usb_queue;
    std::size_t usb_queue_high_watermark {0};
    std::uint64_t usb_dropped_events {0};
    std::thread publisher;
};

IoLoop::IoLoop(const ServiceConfig& config,
               std::atomic<std::uint64_t>& published_packets,
               std::atomic<std::uint64_t>& sequence_counter)
    : impl_(std::make_unique<Impl>(config, published_packets, sequence_counter)) {}

IoLoop::~IoLoop() {
    stop();
}

IoLoop::IoLoop(IoLoop&&) noexcept = default;
IoLoop& IoLoop::operator=(IoLoop&&) noexcept = default;

void IoLoop::start() {
    if (impl_) {
        impl_->start();
    }
}

void IoLoop::stop() {
    if (impl_) {
        impl_->stop();
    }
}

void IoLoop::enqueue(MidiPacket packet) {
    if (impl_) {
        impl_->enqueue(std::move(packet));
    }
}

bool IoLoop::send_midi(std::size_t global_port, std::vector<std::uint8_t> bytes, std::string& error) {
    if (!impl_) {
        error = "io loop not initialized";
        return false;
    }
    return impl_->send_midi(global_port, std::move(bytes), error);
}

IoMetrics IoLoop::snapshot() const {
    if (!impl_) {
        return {};
    }
    return impl_->snapshot();
}

}  // namespace raptor::midi_io
