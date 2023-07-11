add_library(ugps INTERFACE)

target_sources(ugps INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/ugps.cpp
)

target_link_libraries(ugps
        INTERFACE
        pico_stdlib
        hardware_uart
        gnss
)
