add_library(pico-bno08x INTERFACE)

target_sources(pico-bno08x INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/pico-bno08x.cpp
    ${CMAKE_CURRENT_LIST_DIR}/i2c_write_nors_blocking.cpp
)

target_link_libraries(pico-bno08x
    INTERFACE
    pico_stdlib
    hardware_i2c
)