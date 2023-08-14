add_executable(setDirForwardGuide setDirForwardGuide.cpp)

target_link_libraries(setDirForwardGuide 
    pico_stdlib
    hardware_uart
    umotor
    kaguya-pin
)


pico_enable_stdio_usb(setDirForwardGuide 1)
pico_enable_stdio_uart(setDirForwardGuide 0)

pico_add_extra_outputs(setDirForwardGuide)
