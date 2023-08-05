add_executable(readlog2 readlog2.c)

target_link_libraries(readlog2 pico_stdlib hardware_i2c eeprom)

pico_enable_stdio_usb(readlog2 1)
pico_enable_stdio_uart(readlog2 0)

pico_add_extra_outputs(readlog2)