add_executable(monitor_IV monitor_IV.c)

#target_link_libraries(monitor_IV pico_stdlib hardware_gpio health_monitor hardware_pwm hardware_pio motor_encoder)
target_link_libraries(monitor_IV pico_stdlib hardware_gpio health_monitor)


pico_enable_stdio_usb(monitor_IV 1)
pico_enable_stdio_uart(monitor_IV 0)

pico_add_extra_outputs(monitor_IV)
