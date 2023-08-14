add_executable(dir_forward dir_forward.cpp)

target_link_libraries(dir_forward pico_stdlib 
                                  kaguya-pin
                                  umotor
                                  pico-bno08x
                                  )

pico_enable_stdio_usb(dir_forward 1)
pico_enable_stdio_uart(dir_forward 0)

pico_add_extra_outputs(dir_forward)