add_executable(main main.cpp)

# Add include paths
target_include_directories(main
    PUBLIC ${CMAKE_SOURCE_DIR}
    PUBLIC ${CMAKE_SOURCE_DIR}/lib/
)

# pull in common dependencies
target_link_libraries(main PUBLIC
    pico_stdlib
    pico_multicore
    ins
    control
    transmitter
    datalogger
    test_module
)

# Stdio output is sent to USB for debugging instead of UART.
pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(main)
