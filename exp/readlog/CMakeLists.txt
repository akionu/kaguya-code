add_executable(readlog readlog.c)

target_link_libraries(readlog pico_stdlib hardware_i2c eeprom)

pico_enable_stdio_usb(readlog 1)
pico_enable_stdio_uart(readlog 0)

pico_add_extra_outputs(readlog)