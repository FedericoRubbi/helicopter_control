add_library(ins ins.c ins.h imu.c imu.h us_sensor.c us_sensor.h)

# pull in common dependencies
target_link_libraries(ins PUBLIC
    pico_stdlib
    hardware_i2c
)

# Stdio output is sent to USB for debugging instead of UART.
pico_enable_stdio_usb(ins 1)
pico_enable_stdio_uart(ins 0)
