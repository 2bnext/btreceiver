# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

# Pull in pico-extras for pico_audio_i2s (I2S driver for PCM5102).
# pico-extras hooks itself into pico_sdk_init() via PICO_SDK_POST_LIST_FILES,
# so it must be added BEFORE pico_sdk_init() runs.
include(FetchContent)

# CMake 3.30+ deprecates the direct FetchContent_Populate form we use
# below (we need populate-without-add_subdirectory so we can call
# add_subdirectory ourselves *after* project()). Opt in to the OLD
# behavior so newer CMake versions don't warn / error.
if(POLICY CMP0169)
    cmake_policy(SET CMP0169 OLD)
endif()

FetchContent_Declare(
    pico_extras
    GIT_REPOSITORY https://github.com/raspberrypi/pico-extras.git
    GIT_TAG sdk-2.2.0
)
FetchContent_GetProperties(pico_extras)
if(NOT pico_extras_POPULATED)
    FetchContent_Populate(pico_extras)
endif()
set(PICO_EXTRAS_PATH ${pico_extras_SOURCE_DIR})

project(btspkr C CXX ASM)

# Adds pico-extras' post_init.cmake to PICO_SDK_POST_LIST_FILES so its
# libraries (pico_audio_i2s etc.) get registered during pico_sdk_init().
add_subdirectory(${pico_extras_SOURCE_DIR} ${pico_extras_BINARY_DIR})

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# I2S pin assignment for PCM5102 DAC module.
# DATA_PIN    = DIN  (GPIO26)
# CLOCK_BASE  = BCK  (GPIO27); LRCK = BCK + 1 = GPIO28
set(BTSPKR_I2S_DATA_PIN       26)
set(BTSPKR_I2S_CLOCK_PIN_BASE 27)

add_executable(btspkr
    btspkr.cpp
    audio.cpp
    bt.cpp
    dsp.cpp
    pots.cpp
    vu.cpp
    btstack_audio_pico.c
    ${PICO_BTSTACK_PATH}/src/btstack_audio.c
)

pico_set_program_name(btspkr "btspkr")
pico_set_program_version(btspkr "0.1")

# stdio over UART so prints survive debugger halts (the probe forwards
# GPIO 0/1 to its own USB-CDC, independent of TinyUSB on the target).
pico_enable_stdio_uart(btspkr 1)
pico_enable_stdio_usb(btspkr 0)

target_compile_definitions(btspkr PRIVATE
    PICO_AUDIO_I2S_DATA_PIN=${BTSPKR_I2S_DATA_PIN}
    PICO_AUDIO_I2S_CLOCK_PIN_BASE=${BTSPKR_I2S_CLOCK_PIN_BASE}
    CYW43_LWIP=0
)

# Our own btstack_config.h lives next to btspkr.cpp
target_include_directories(btspkr PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(btspkr
    pico_stdlib
    hardware_i2c
    hardware_pwm
    pico_cyw43_arch_threadsafe_background
    pico_btstack_classic
    pico_btstack_sbc_decoder
    pico_btstack_cyw43
    pico_audio_i2s
)

pico_add_extra_outputs(btspkr)
