add_executable(parachute parachute.c)

target_link_libraries(parachute pico_stdlib hardware_i2c eeprom pico-spl06-001)

pico_enable_stdio_usb(parachute 1)
pico_enable_stdio_uart(parachute 0)

pico_add_extra_outputs(parachute)