add_library(gnss INTERFACE)

target_sources(gnss INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/nmeap01.c
)

target_link_libraries(gnss
        INTERFACE
        pico_stdlib
        hardware_uart
)
