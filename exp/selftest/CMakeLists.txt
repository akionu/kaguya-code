add_executable(selftest selftest.c)

target_link_libraries(selftest 
    pico_stdlib
    hardware_gpio
    hardware_pwm
    hardware_pio
    hardware_i2c
    hardware_uart
    prometheus-pin
    motor_encoder
    health_monitor
    pico-icm20948
    eeprom
    vl53l5cx
)


pico_enable_stdio_usb(selftest 1)
pico_enable_stdio_uart(selftest 0)

pico_add_extra_outputs(selftest)
