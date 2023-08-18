add_executable(flight_A flight.cpp)
add_executable(flight_B flight.cpp)
add_executable(flight_C flight.cpp)

target_link_libraries(flight_A
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
    #eeprom 
    #pico-spl06-001 
    uprs
    #vl53l5cx 
    utof
    ulog
)

target_link_libraries(flight_B
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
    #eeprom 
    #pico-spl06-001 
    uprs
    #vl53l5cx 
    utof
    ulog
)

target_link_libraries(flight_C
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
    #eeprom 
    #pico-spl06-001 
    uprs
    #vl53l5cx 
    utof
    ulog
)

target_compile_definitions(flight_A PUBLIC CODE_A)
target_compile_definitions(flight_B PUBLIC CODE_B)
target_compile_definitions(flight_C PUBLIC CODE_C)


pico_enable_stdio_usb(flight_A 1)
pico_enable_stdio_uart(flight_A 0)
pico_add_extra_outputs(flight_A)

pico_enable_stdio_usb(flight_B 1)
pico_enable_stdio_uart(flight_B 0)
pico_add_extra_outputs(flight_B)

pico_enable_stdio_usb(flight_C 1)
pico_enable_stdio_uart(flight_C 0)
pico_add_extra_outputs(flight_C)