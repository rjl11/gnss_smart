# DHCP and DNS servers for the open access point gnss_smart
# (plans/web-config-server.md, Phase 3). Copied unchanged from pico-examples
# (pico_w/wifi/access_point, commit 0d62f75); see README.md.
#
# An INTERFACE library, unlike the other drivers: the sources compile in the
# app target's context, so they see its lwipopts.h and the cyw43 headers.
#
# Usage, after pico_sdk_init():
#   add_subdirectory(drivers/ap_servers)
#   target_link_libraries(app PRIVATE ap_servers)

add_library(ap_servers INTERFACE)
target_sources(ap_servers INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/dhcpserver.c
    ${CMAKE_CURRENT_LIST_DIR}/dnsserver.c
)
target_include_directories(ap_servers INTERFACE ${CMAKE_CURRENT_LIST_DIR})

# Vendor code stays unchanged; quiet the app's -Wextra for it. The sources
# compile in the top-level directory, where the app target lives.
set_source_files_properties(
    ${CMAKE_CURRENT_LIST_DIR}/dhcpserver.c
    ${CMAKE_CURRENT_LIST_DIR}/dnsserver.c
    DIRECTORY ${CMAKE_SOURCE_DIR}
    PROPERTIES COMPILE_OPTIONS "-Wno-unused-parameter")
