add_executable(ex_gnss ex_gnss.c)

target_link_libraries(ex_gnss pico_stdlib hardware_uart gnss)

pico_enable_stdio_usb(ex_gnss 1)
pico_enable_stdio_uart(ex_gnss 0)

pico_add_extra_outputs(ex_gnss)