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

struct IoLoop::Impl {
    struct PublishedItem {
        MidiPacket packet;
    };

    struct BusWorker {
        std::string bus_key;
        std::vector<ModuleConfig> modules;
        std::deque<PublishedItem> queue;
        std::size_t queue_capacity {0};
        std::size_t warning_threshold {0};
        std::size_t queue_high_watermark {0};
        std::uint64_t dropped_events {0};
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
          usb_midi_input(config.usb_midi_controllers, sequence_counter, [this](MidiPacket packet) {
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

    void start_bus_worker(BusWorker& worker) {
        worker.worker = std::thread([this, &worker]() {
            for (const auto& module : worker.modules) {
                worker.gpio.register_module(module, [&](const ModuleConfig& triggered) {
                    auto read_result = worker.spi.read_packet(triggered);
                    if (read_result.bytes.empty()) {
                        return;
                    }

                    if (read_result.local_port < 1 || read_result.local_port > triggered.midi_port_count) {
                        spdlog::debug(
                            "spi packet dropped invalid local_port={} module={} configured_ports={}",
                            read_result.local_port,
                            triggered.id,
                            triggered.midi_port_count);
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

            while (!stop_requested.load(std::memory_order_relaxed)) {
                worker.gpio.run_once();
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        });
    }

    void publisher_loop() {
        EventBus bus {config.ipc.events_endpoint};
        spdlog::debug("io publisher start endpoint={}", config.ipc.events_endpoint);
        std::size_t next_bus_index = 0;

        while (true) {
            std::unique_lock<std::mutex> lock(queue_mutex);
            queue_cv.wait(lock, [&]() {
                return stop_requested.load(std::memory_order_relaxed) || has_pending_items_locked();
            });

            if (stop_requested.load(std::memory_order_relaxed) && !has_pending_items_locked()) {
                break;
            }

            PublishedItem item;
            bool found = false;

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
            lock.unlock();

            if (!found) {
                continue;
            }

            bus.publish(item.packet);
            published_packets.fetch_add(1, std::memory_order_relaxed);
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
            if (w.dropped_events > 0 || w.queue_high_watermark > 0) {
                spdlog::debug(
                    "io stop bus={} dropped_events={} high_watermark={}",
                    w.bus_key,
                    w.dropped_events,
                    w.queue_high_watermark);
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
            });
        }

        for (const auto& worker : bus_workers) {
            metrics.dropped_events_total += worker.dropped_events;
            metrics.buses.push_back(BusIoMetrics {
                .bus_key = worker.bus_key,
                .module_count = worker.modules.size(),
                .queue_depth = worker.queue.size(),
                .queue_capacity = worker.queue_capacity,
                .queue_high_watermark = worker.queue_high_watermark,
                .dropped_events = worker.dropped_events,
            });
        }

        return metrics;
    }

    ServiceConfig config;
    std::atomic<std::uint64_t>& published_packets;
    std::atomic<std::uint64_t>& sequence_counter;
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

IoMetrics IoLoop::snapshot() const {
    if (!impl_) {
        return {};
    }
    return impl_->snapshot();
}

}  // namespace raptor::midi_io
