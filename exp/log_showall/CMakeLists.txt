add_executable(log_showall log_showall.cpp)

target_link_libraries(log_showall pico_stdlib hardware_i2c ulog kaguya-pin)

pico_enable_stdio_usb(log_showall 1)
pico_enable_stdio_uart(log_showall 0)

pico_add_extra_outputs(log_showall)