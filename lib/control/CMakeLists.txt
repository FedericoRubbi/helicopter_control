add_library(control control.c control.h)

# Add include paths
target_include_directories(control
    PUBLIC ${CMAKE_SOURCE_DIR}/lib/
)

# pull in common dependencies
target_link_libraries(control PUBLIC
    pico_stdlib
)

# Stdio output is sent to USB for debugging instead of UART.
pico_enable_stdio_usb(control 1)
pico_enable_stdio_uart(control 0)
