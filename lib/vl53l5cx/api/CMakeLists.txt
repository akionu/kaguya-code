add_library(vl53l5cx INTERFACE)

target_sources(vl53l5cx INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/platform.c
    ${CMAKE_CURRENT_LIST_DIR}/vl53l5cx_api.c
    ${CMAKE_CURRENT_LIST_DIR}/vl53l5cx_plugin_detection_thresholds.c
    ${CMAKE_CURRENT_LIST_DIR}/vl53l5cx_plugin_motion_indicator.c
    ${CMAKE_CURRENT_LIST_DIR}/vl53l5cx_plugin_xtalk.c
)

target_link_libraries(vl53l5cx
    INTERFACE
    pico_stdlib
    hardware_i2c
)
