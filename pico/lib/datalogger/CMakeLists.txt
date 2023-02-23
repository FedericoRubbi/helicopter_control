add_library(datalogger datalogger.c datalogger.h)

target_include_directories(datalogger
    PUBLIC ${CMAKE_SOURCE_DIR}/lib/
)

# pull in common dependencies
target_link_libraries(datalogger PRIVATE
    pico_stdlib
)

# Stdio output is sent to USB for debugging instead of UART.
pico_enable_stdio_usb(datalogger 1)
pico_enable_stdio_uart(datalogger 0)
