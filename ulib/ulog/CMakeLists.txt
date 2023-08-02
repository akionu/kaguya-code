add_library(ulog INTERFACE)

target_sources(ulog INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/ulog.cpp
)

target_link_libraries(ulog
        INTERFACE
        pico_stdlib
        hardware_rtc
        hardware_uart
        pico_e220
        eeprom
        kaguya-pin
)
