add_executable(rotate rotate.c lora.c)

target_link_libraries(rotate pico_stdlib hardware_uart kaguya-pin)

pico_enable_stdio_usb(rotate 1)
pico_enable_stdio_uart(rotate 0)

pico_add_extra_outputs(rotate)

add_executable(child child.c lora.c)
target_link_libraries(child pico_stdlib hardware_uart motor_encoder kaguya-pin)

pico_enable_stdio_usb(child 1)
pico_enable_stdio_uart(child 0)

pico_add_extra_outputs(child)