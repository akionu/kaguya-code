add_library(health_monitor INTERFACE)

target_sources(health_monitor INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/health.c
)

target_link_libraries(health_monitor
        INTERFACE
        pico_stdlib
        hardware_gpio
        hardware_adc
)
