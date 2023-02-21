cmake_minimum_required(VERSION 3.13.0)


# Setup pico SDK.
set(ENV{PICO_SDK_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/lib/pico-sdk")
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()


# Setup project.
project(heli_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
enable_language( C CXX ASM )
# NOTE: -DPICO_BOARD=waveshare_rp2040_zero is set in workspace settings.json.
add_compile_options(
    -Wall
    -Wno-format
    -Wno-unused-function
    -Wno-maybe-uninitialized
    -Wno-comment
)


# Initialize pico SDK.
pico_sdk_init()


# Add subdirectories.
add_subdirectory(src)
add_subdirectory(lib)
