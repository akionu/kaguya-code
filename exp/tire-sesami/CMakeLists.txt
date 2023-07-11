add_executable(tire-sesami tire-sesami.c)

target_link_libraries(tire-sesami
                        pico_stdlib
                        hardware_pwm
                        motor_encoder
                        kaguya-pin)

pico_enable_stdio_uart(tire-sesami 0)
pico_enable_stdio_usb(tire-sesami 1)
pico_add_extra_outputs(tire-sesami)