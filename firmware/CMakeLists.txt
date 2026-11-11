# gnss_smart firmware: sensor bring-up + INSLIB nav_suite demo (Pico 2 W, bare metal).
#
# Build (PICO_SDK_PATH set, see CLAUDE.md for the miniPi rules):
#   cmake -S firmware -B firmware/build -G Ninja
#   ninja -C firmware/build
# Output: firmware/build/app.uf2 (and app.elf for SWD).

cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico2_w CACHE STRING "Board type")
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

include(pico_sdk_import.cmake)

project(gnss_smart C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_subdirectory(drivers/uart_dma)
add_subdirectory(drivers/bmp581)
add_subdirectory(drivers/tm210)
add_subdirectory(drivers/um980)
add_subdirectory(drivers/inslib)

add_executable(app
    app/main.c
    app/nav_core1.c
)

target_include_directories(app PRIVATE app)

target_compile_options(app PRIVATE -Wall -Wextra)

target_link_libraries(app PRIVATE
    pico_stdlib
    pico_multicore
    pico_util
    pico_sync
    hardware_uart
    hardware_i2c
    uart_dma
    bmp581
    tm210
    um980
    inslib
)

# The whole image runs from SRAM, so the filter's timing does not depend on
# the XIP cache (plans/rp2350-feasibility.md, risk 4).
pico_set_binary_type(app copy_to_ram)

# Debug output on USB CDC. UART0 is the TM210, so stdio must never use it.
pico_enable_stdio_usb(app 1)
pico_enable_stdio_uart(app 0)

pico_add_extra_outputs(app)
