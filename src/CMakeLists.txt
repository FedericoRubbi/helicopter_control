add_executable(main main.cpp)

# pull in common dependencies
target_link_libraries(main PUBLIC
    RF24
    pico_stdlib
    hardware_spi
    hardware_gpio
)

# Stdio output is sent to USB for debugging instead of UART.
pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(main)
