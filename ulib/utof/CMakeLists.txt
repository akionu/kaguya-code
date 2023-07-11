add_library(utof INTERFACE)

target_sources(utof INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/utof.cpp
)

target_link_libraries(utof
        INTERFACE
        pico_stdlib
        hardware_i2c
        vl53l5cx
)
