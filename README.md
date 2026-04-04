# raptor-midi-io-service

Daemon for Raspberry Pi that reads MIDI packets from multiple ESP-based MIDI I/O modules over a shared SPI bus and from configured USB MIDI controllers.

## Architectural direction

- One daemon owns the MIDI upstream for the whole local system.
- ESP modules deliver MIDI over SPI.
- USB controllers deliver MIDI through ALSA sequencer input.
- Both sources are normalized into the same `midi.packet` ZeroMQ event bus.
- Sequencer and UI consume one stream and do not need separate source-specific subscribers.

## Current implementation status

- SPI backend is implemented with `spidev`
- GPIO handshake and software chip-select use `libgpiod`
- handshake detection uses `libgpiod` line events
- dedicated I/O scheduler groups SPI modules by `spi_device`
- a publisher thread owns ZeroMQ event publication
- bounded queues use `drop_oldest`
- configuration is loaded with `yaml-cpp`
- JSON parsing and serialization use `nlohmann_json`
- global MIDI port numbering is derived from SPI module order
- USB MIDI upstream is now implemented through ALSA sequencer and publishes on the same `midi.packet` bus

## IPC choice

Use ZeroMQ everywhere for local IPC.

Suggested split:

- `ipc:///run/raptor-midi-io/events.zmq`: daemon publishes MIDI packets from SPI and USB sources
- `ipc:///run/raptor-midi-io/control.zmq`: daemon exposes control, health, diagnostics, and runtime commands

## Event schema

MIDI events use JSON over ZeroMQ `PUB`. The first frame is the topic `midi.packet`, and the second frame is a JSON payload.

SPI example:

```json
{
  "schema_version": "1.0",
  "service": "raptor-midi-io-service",
  "type": "midi.packet",
  "sequence": 42,
  "timestamp_ns": 1234567890123,
  "source": {
    "source_kind": "spi-module",
    "module_id": "esp-a",
    "controller_id": "",
    "device_name": "",
    "spi_device": "/dev/spidev0.0",
    "spi_speed_hz": 1000000,
    "spi_mode": 0,
    "chip_select_gpio": 17,
    "handshake_gpio": 23,
    "handshake_active_low": false,
    "module_port_count": 2,
    "module_first_global_port": 1,
    "module_last_global_port": 2,
    "local_port": 2,
    "global_port": 2
  },
  "midi": {
    "size": 3,
    "bytes": [144, 60, 127],
    "bytes_hex": "90 3C 7F"
  }
}
```

USB controller example:

```json
{
  "schema_version": "1.0",
  "service": "raptor-midi-io-service",
  "type": "midi.packet",
  "sequence": 314,
  "timestamp_ns": 1234567890999,
  "source": {
    "source_kind": "usb-midi-controller",
    "module_id": "x-touch-mini-a",
    "controller_id": "x-touch-mini-a",
    "device_name": "X-TOUCH MINI:X-TOUCH MINI MIDI 1",
    "spi_device": "",
    "spi_speed_hz": 0,
    "spi_mode": 0,
    "chip_select_gpio": -1,
    "handshake_gpio": -1,
    "handshake_active_low": false,
    "module_port_count": 1,
    "module_first_global_port": 0,
    "module_last_global_port": 0,
    "local_port": 1,
    "global_port": 0
  },
  "midi": {
    "size": 3,
    "bytes": [176, 20, 127],
    "bytes_hex": "B0 14 7F"
  }
}
```

This schema lets downstream services distinguish source kinds while still consuming one common topic.

## USB MIDI upstream

Configured controllers live under `usb_midi_controllers:` in YAML. Each enabled controller is matched against ALSA sequencer client/port names via `match_name`.

When a matching ALSA sequencer source is found, the daemon subscribes to it and republishes incoming MIDI as `midi.packet` on the common ZeroMQ bus.

This is the intended upstream path for controllers such as:
- `X-TOUCH MINI`
- USB pad controllers
- generic class-compliant USB MIDI devices

## Build

Native:

```sh
cmake -S . -B build
cmake --build build
```

On Debian-like hosts you will want:
- `libzmq3-dev`
- `libgpiod-dev`
- `libyaml-cpp-dev`
- `nlohmann-json3-dev`
- `libasound2-dev`
