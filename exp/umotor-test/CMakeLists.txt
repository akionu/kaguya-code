add_executable(umotor-test umotor-test.cpp)

target_link_libraries(umotor-test
                        pico_stdlib
                        umotor
                        kaguya-pin
                        )

pico_enable_stdio_uart(umotor-test 0)
pico_enable_stdio_usb(umotor-test 1)
pico_add_extra_outputs(umotor-test)