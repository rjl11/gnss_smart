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

# GNSS antenna mode default: single (UM980, or UM982 with the slave antenna
# off) or dual (UM982 with heading). See app/app_config.h.
set(GNSS_ANTENNA_MODE single CACHE STRING "GNSS antenna mode default: single or dual")
set_property(CACHE GNSS_ANTENNA_MODE PROPERTY STRINGS single dual)
if(GNSS_ANTENNA_MODE STREQUAL "dual")
    target_compile_definitions(app PRIVATE APP_GNSS_ANTENNA_MODE=1)
elseif(GNSS_ANTENNA_MODE STREQUAL "single")
    target_compile_definitions(app PRIVATE APP_GNSS_ANTENNA_MODE=0)
else()
    message(FATAL_ERROR "GNSS_ANTENNA_MODE must be single or dual, not '${GNSS_ANTENNA_MODE}'")
endif()

# Wi-Fi and the web server (plans/web-config-server.md, "When Wi-Fi runs").
# always: station + status page from boot (the development default).
# off: no Wi-Fi code linked in. switch (GPIO22 at boot) comes in Phase 5.
set(WEB_MODE always CACHE STRING "Web server: always or off")
set_property(CACHE WEB_MODE PROPERTY STRINGS always off)
set(WIFI_STA_SSID "TeliaGateway" CACHE STRING "Wi-Fi network to join")
set(WIFI_STA_PASSWORD "" CACHE STRING "Wi-Fi password; empty for an open network")
# auto: station, open AP gnss_smart if not joined within 30 s of boot.
# sta: station only. ap: open AP from boot.
set(WIFI_MODE auto CACHE STRING "Wi-Fi network mode: auto, sta or ap")
set_property(CACHE WIFI_MODE PROPERTY STRINGS auto sta ap)
if(WIFI_MODE STREQUAL "auto")
    set(APP_WIFI_MODE_VALUE 0)
elseif(WIFI_MODE STREQUAL "sta")
    set(APP_WIFI_MODE_VALUE 1)
elseif(WIFI_MODE STREQUAL "ap")
    set(APP_WIFI_MODE_VALUE 2)
else()
    message(FATAL_ERROR "WIFI_MODE must be auto, sta or ap, not '${WIFI_MODE}'")
endif()
if(WEB_MODE STREQUAL "always")
    add_subdirectory(drivers/ap_servers)
    target_sources(app PRIVATE app/web/web.c app/web/jsonw.c)
    target_link_libraries(app PRIVATE ap_servers)
    target_compile_definitions(app PRIVATE
        APP_WEB_MODE=1
        APP_WIFI_MODE=${APP_WIFI_MODE_VALUE}
        "APP_WIFI_STA_SSID=\"${WIFI_STA_SSID}\""
        "APP_WIFI_STA_PASSWORD=\"${WIFI_STA_PASSWORD}\""
        "CYW43_HOST_NAME=\"gnss-smart\""
    )
    target_link_libraries(app PRIVATE
        pico_cyw43_arch_lwip_threadsafe_background
        pico_lwip_http
        pico_lwip_mdns
    )
    pico_set_lwip_httpd_content(app PRIVATE ${CMAKE_CURRENT_LIST_DIR}/app/web/www/index.html)
elseif(WEB_MODE STREQUAL "off")
    target_compile_definitions(app PRIVATE APP_WEB_MODE=0)
elseif(WEB_MODE STREQUAL "switch")
    message(FATAL_ERROR "WEB_MODE=switch is Phase 5 and not implemented yet")
else()
    message(FATAL_ERROR "WEB_MODE must be always or off, not '${WEB_MODE}'")
endif()

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

# Default flash (XIP) binary. A whole-image copy_to_ram build won't fit once
# the CYW43 Wi-Fi firmware and lwIP are linked in, so only the core 1 filter
# path (INSLIB, nav_core1.c and the SDK calls it makes) goes to SRAM, through
# the linker-script override in link/. The timestamping ISRs are marked
# __not_in_flash_func in their drivers. See plans/web-config-server.md,
# "Memory layout", and check with tools/check_ram_path.py.
pico_add_linker_script_override_path(app ${CMAKE_CURRENT_LIST_DIR}/link
    FILES default_text_excludes.incl default_rodata_excludes.incl)

# Debug output on USB CDC. UART0 is the TM210, so stdio must never use it.
pico_enable_stdio_usb(app 1)
pico_enable_stdio_uart(app 0)

pico_add_extra_outputs(app)
