add_library(test_module test_module.cpp test_module.h)

# Add include paths
target_include_directories(test_module
    PUBLIC ${CMAKE_SOURCE_DIR}/lib/
)

# pull in common dependencies
target_link_libraries(test_module PUBLIC
    pico_stdlib
    hardware_pwm
)

# Stdio output is sent to USB for debugging instead of UART.
pico_enable_stdio_usb(test_module 1)
pico_enable_stdio_uart(test_module 0)
