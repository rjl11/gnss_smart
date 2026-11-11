# SYD Dynamics TM210 IMU driver (Pico SDK): EasyProtocol codec, boot-time
# check/provision state machine and raw-data parser. The vendor library in
# external/syd_dynamics is the protocol reference only and is not compiled.
#
# Usage from the firmware project, after pico_sdk_init():
#   add_subdirectory(drivers/uart_dma)
#   add_subdirectory(drivers/tm210)
#   target_link_libraries(app PRIVATE tm210)

add_library(tm210 STATIC
    ${CMAKE_CURRENT_LIST_DIR}/tm210.c
)

target_include_directories(tm210 PUBLIC ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(tm210
    PUBLIC  uart_dma
    PRIVATE pico_time pico_stdio
)
