add_executable(flight 
    flight.cpp 
)

target_link_libraries(flight 
    pico_stdlib 
    hardware_i2c 
    #gnss
    ugps 
    health_monitor 
    kaguya-pin
    #motor_encoder 
    umotor
    pico-bno08x 
    pico_e220 
    eeprom 
    #pico-spl06-001 
    uprs
    #vl53l5cx 
    utof
)

pico_enable_stdio_usb(flight 1)
pico_enable_stdio_uart(flight 0)

pico_add_extra_outputs(flight)