add_library(motor_encoder INTERFACE)

pico_generate_pio_header(motor_encoder ${CMAKE_CURRENT_LIST_DIR}/quadrature_encoder.pio)

target_sources(motor_encoder INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/motor.c
)

target_link_libraries(motor_encoder
        INTERFACE
        pico_stdlib
        pico_multicore
        hardware_pio
        hardware_pwm
)
