add_executable(cone cone.c)

target_link_libraries(cone pico_stdlib hardware_i2c vl53l5cx kaguya-pin)

pico_enable_stdio_usb(cone 1)
pico_enable_stdio_uart(cone 0)

pico_add_extra_outputs(cone)