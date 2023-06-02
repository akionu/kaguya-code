add_executable(nichrome nichrome.c)

target_link_libraries(nichrome pico_stdlib)

pico_enable_stdio_usb(nichrome 1)
pico_enable_stdio_uart(nichrome 0)

pico_add_extra_outputs(nichrome)