add_executable(main main.cpp)

# Add include paths
include_directories(
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/lib
)

# pull in common dependencies
target_link_libraries(main PRIVATE
    pico_stdlib
)

# Stdio output is sent to USB for debugging instead of UART.
pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(main)
