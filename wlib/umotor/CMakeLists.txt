add_library(umotor INTERFACE)

target_sources(umotor INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/umotor.cpp
)

target_link_libraries(umotor
        INTERFACE
        pico_stdlib
        hardware_pwm
        motor_encoder
)
