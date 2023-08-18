add_executable(receiver_A receiver.c)
target_link_libraries(receiver_A pico_stdlib pico_e220 kaguya-pin)
target_compile_definitions(receiver_A PUBLIC CODE_A)
pico_enable_stdio_usb(receiver_A 1)
pico_enable_stdio_uart(receiver_A 0)
pico_add_extra_outputs(receiver_A)

add_executable(receiver_B receiver.c)
target_link_libraries(receiver_B pico_stdlib pico_e220 kaguya-pin)
target_compile_definitions(receiver_B PUBLIC CODE_B)
pico_enable_stdio_usb(receiver_B 1)
pico_enable_stdio_uart(receiver_B 0)
pico_add_extra_outputs(receiver_B)

add_executable(receiver_C receiver.c)
target_link_libraries(receiver_C pico_stdlib pico_e220 kaguya-pin)
target_compile_definitions(receiver_C PUBLIC CODE_C)
pico_enable_stdio_usb(receiver_C 1)
pico_enable_stdio_uart(receiver_C 0)
pico_add_extra_outputs(receiver_C)

add_executable(tranceiver tranceiver.c)

target_link_libraries(tranceiver pico_stdlib gnss pico_e220 kaguya-pin hardware_rtc)

pico_enable_stdio_usb(tranceiver 1)
pico_enable_stdio_uart(tranceiver 0)

pico_add_extra_outputs(tranceiver)