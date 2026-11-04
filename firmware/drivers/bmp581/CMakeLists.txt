# BMP581 barometer driver (Pico SDK) on top of Bosch's BMP5 Sensor API.
#
# Usage from the firmware project, after pico_sdk_init():
#   add_subdirectory(drivers/bmp581)
#   target_link_libraries(app PRIVATE bmp581)

set(BMP5_API_DIR ${CMAKE_CURRENT_LIST_DIR}/../../../external/BMP5_SensorAPI)

add_library(bmp581 STATIC
    ${CMAKE_CURRENT_LIST_DIR}/bmp581.c
    ${BMP5_API_DIR}/bmp5.c
)

target_include_directories(bmp581 PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${BMP5_API_DIR}
)

# RP2350 has a hardware FPU, so use the API's float output (Pa, degC).
target_compile_definitions(bmp581 PUBLIC BMP5_USE_FLOATING_POINT)

target_link_libraries(bmp581
    PUBLIC  hardware_i2c
    PRIVATE hardware_gpio hardware_irq hardware_sync pico_time
)
