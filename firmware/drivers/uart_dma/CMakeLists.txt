# UART with DMA ring-buffer receive and a polled transmit ring (Pico SDK, RP2350).
#
# Usage from the firmware project, after pico_sdk_init():
#   add_subdirectory(drivers/uart_dma)
#   target_link_libraries(app PRIVATE uart_dma)

add_library(uart_dma STATIC
    ${CMAKE_CURRENT_LIST_DIR}/uart_dma.c
)

target_include_directories(uart_dma PUBLIC ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(uart_dma
    PUBLIC  hardware_uart
    PRIVATE hardware_dma hardware_gpio pico_time
)
