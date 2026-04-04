cmake_minimum_required(VERSION 3.21)

project(raptor-midi-io-service VERSION 0.1.0 LANGUAGES CXX)

option(RAPTOR_MIDI_IO_BUILD_SERVICE "Build the background service" ON)
option(RAPTOR_MIDI_IO_INSTALL_SYSTEMD_UNIT "Install the systemd service file" ON)
option(RAPTOR_MIDI_IO_INSTALL_DEFAULT_CONFIG "Install the default YAML configuration file" ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(GNUInstallDirs)
include(CheckIncludeFileCXX)

find_package(Threads REQUIRED)
find_package(PkgConfig QUIET)
find_package(nlohmann_json CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(yaml-cpp CONFIG REQUIRED)

option(RAPTOR_MIDI_IO_USE_ZEROMQ "Use ZeroMQ for IPC" ON)
option(RAPTOR_MIDI_IO_USE_GPIOD "Use libgpiod for GPIO access" ON)

if(PkgConfig_FOUND)
    if(RAPTOR_MIDI_IO_USE_ZEROMQ)
        pkg_check_modules(LIBZMQ IMPORTED_TARGET libzmq)
    endif()
    if(RAPTOR_MIDI_IO_USE_GPIOD)
        pkg_check_modules(LIBGPIOD IMPORTED_TARGET libgpiod)
    endif()
    pkg_check_modules(ALSA REQUIRED IMPORTED_TARGET alsa)
endif()

add_library(raptor_midi_io_core
    src/application.cpp
    src/config.cpp
    src/control_server.cpp
    src/event_bus.cpp
    src/gpio_manager.cpp
    src/io_loop.cpp
    src/logger.cpp
    src/spi_bus.cpp
    src/usb_midi_input.cpp
)

target_include_directories(raptor_midi_io_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_compile_features(raptor_midi_io_core PUBLIC cxx_std_20)

target_compile_options(raptor_midi_io_core PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic>
)

target_link_libraries(raptor_midi_io_core PUBLIC Threads::Threads)
target_link_libraries(raptor_midi_io_core PUBLIC nlohmann_json::nlohmann_json)
target_link_libraries(raptor_midi_io_core PUBLIC spdlog::spdlog)
target_link_libraries(raptor_midi_io_core PUBLIC yaml-cpp)
target_link_libraries(raptor_midi_io_core PUBLIC PkgConfig::ALSA)

if(RAPTOR_MIDI_IO_USE_ZEROMQ)
    if(TARGET PkgConfig::LIBZMQ)
        target_link_libraries(raptor_midi_io_core PUBLIC PkgConfig::LIBZMQ)
        target_compile_definitions(raptor_midi_io_core PUBLIC RAPTOR_MIDI_IO_HAS_ZEROMQ=1)
    else()
        message(WARNING "ZeroMQ requested but libzmq was not found. Building transport stub only.")
        target_compile_definitions(raptor_midi_io_core PUBLIC RAPTOR_MIDI_IO_HAS_ZEROMQ=0)
    endif()
else()
    target_compile_definitions(raptor_midi_io_core PUBLIC RAPTOR_MIDI_IO_HAS_ZEROMQ=0)
endif()

if(RAPTOR_MIDI_IO_USE_GPIOD)
    if(TARGET PkgConfig::LIBGPIOD)
        target_link_libraries(raptor_midi_io_core PUBLIC PkgConfig::LIBGPIOD)
        target_compile_definitions(raptor_midi_io_core PUBLIC RAPTOR_MIDI_IO_HAS_GPIOD=1)
    else()
        message(WARNING "libgpiod requested but not found. Building GPIO fallback only.")
        target_compile_definitions(raptor_midi_io_core PUBLIC RAPTOR_MIDI_IO_HAS_GPIOD=0)
    endif()
else()
    target_compile_definitions(raptor_midi_io_core PUBLIC RAPTOR_MIDI_IO_HAS_GPIOD=0)
endif()

if(RAPTOR_MIDI_IO_BUILD_SERVICE)
    add_executable(raptor-midi-io-service src/main.cpp)
    target_link_libraries(raptor-midi-io-service PRIVATE raptor_midi_io_core)
endif()

install(TARGETS raptor_midi_io_core
    EXPORT raptor-midi-io-service-targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

if(RAPTOR_MIDI_IO_BUILD_SERVICE)
    install(TARGETS raptor-midi-io-service
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
endif()

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

if(RAPTOR_MIDI_IO_INSTALL_DEFAULT_CONFIG)
    install(FILES config/raptor-midi-io-service.example.yaml
        DESTINATION ${CMAKE_INSTALL_SYSCONFDIR}/raptor-midi-io-service
        RENAME raptor-midi-io-service.yaml
    )
    install(FILES config/raptor-midi-io-service.example.yaml
        DESTINATION ${CMAKE_INSTALL_SYSCONFDIR}/raptor-midi-io-service
    )
endif()

if(RAPTOR_MIDI_IO_INSTALL_SYSTEMD_UNIT)
    install(FILES systemd/raptor-midi-io-service.service DESTINATION ${CMAKE_INSTALL_LIBDIR}/systemd/system)
endif()