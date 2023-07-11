add_library(uprs INTERFACE)

target_sources(uprs INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/uprs.cpp
)

target_link_libraries(uprs
        INTERFACE
        pico_stdlib
        hardware_i2c
        pico-spl06-001
)
