add_executable(nichrome2 nichrome.c)

target_link_libraries(nichrome2 pico_stdlib)

pico_enable_stdio_usb(nichrome2 1)
pico_enable_stdio_uart(nichrome2 0)

pico_add_extra_outputs(nichrome2)