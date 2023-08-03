add_executable(kaisan kaisan.cpp)

target_link_libraries(kaisan pico_stdlib hardware_i2c pico-bno08x eeprom kaguya-pin)

pico_enable_stdio_usb(kaisan 1)
pico_enable_stdio_uart(kaisan 0)

pico_add_extra_outputs(kaisan)

add_executable(readlog readlog.cpp)

target_link_libraries(readlog pico_stdlib hardware_i2c eeprom kaguya-pin)

pico_enable_stdio_usb(readlog 1)
pico_enable_stdio_uart(readlog 0)

pico_add_extra_outputs(readlog)