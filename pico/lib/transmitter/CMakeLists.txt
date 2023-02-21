add_library(transmitter transmitter.cpp transmitter.h)

target_include_directories(transmitter
    PUBLIC ${CMAKE_SOURCE_DIR}/lib/
)

# pull in common dependencies
target_link_libraries(transmitter PRIVATE
    pico_stdlib
    hardware_pwm
    hardware_spi
    hardware_gpio
    RF24
)

# Stdio output is sent to USB for debugging instead of UART.
pico_enable_stdio_usb(transmitter 1)
pico_enable_stdio_uart(transmitter 0)
