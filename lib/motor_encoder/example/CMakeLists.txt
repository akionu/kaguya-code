add_executable(ex_motor_encoder)

target_sources(ex_motor_encoder PRIVATE ex_motor_encoder.c)

target_link_libraries(ex_motor_encoder PRIVATE
        pico_stdlib
        pico_multicore
        hardware_pio
        hardware_pwm
        motor_encoder
)

pico_enable_stdio_usb(ex_motor_encoder 1)
pico_enable_stdio_uart(ex_motor_encoder 0)

pico_add_extra_outputs(ex_motor_encoder)