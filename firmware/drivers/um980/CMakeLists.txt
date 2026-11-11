# Unicore UM980 GNSS receiver driver (Pico SDK): binary log parser, PPS
# time-tagging and the boot-time check/provision state machine.
#
# Usage from the firmware project, after pico_sdk_init():
#   add_subdirectory(drivers/uart_dma)
#   add_subdirectory(drivers/um980)
#   target_link_libraries(app PRIVATE um980)

add_library(um980 STATIC
    ${CMAKE_CURRENT_LIST_DIR}/um980.c
    ${CMAKE_CURRENT_LIST_DIR}/um980_cfg.c
)

target_include_directories(um980 PUBLIC ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(um980
    PUBLIC  uart_dma
    PRIVATE hardware_gpio hardware_irq hardware_sync pico_time pico_stdio
)
