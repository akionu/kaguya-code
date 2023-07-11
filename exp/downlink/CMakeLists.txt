add_executable(receiver receiver.c)

target_link_libraries(receiver pico_stdlib pico_e220 kaguya-pin)

pico_enable_stdio_usb(receiver 1)
pico_enable_stdio_uart(receiver 0)

pico_add_extra_outputs(receiver)

add_executable(tranceiver tranceiver.c)

target_link_libraries(tranceiver pico_stdlib gnss pico_e220 kaguya-pin hardware_rtc)

pico_enable_stdio_usb(tranceiver 1)
pico_enable_stdio_uart(tranceiver 0)

pico_add_extra_outputs(tranceiver)