# INSLIB nav_suite (external/INSLIB) as a Pico SDK static library.
# No glue code: the library has no heap and no OS dependencies.
#
# Usage from the firmware project, after pico_sdk_init():
#   add_subdirectory(drivers/inslib)
#   target_link_libraries(app PRIVATE inslib)

set(INSLIB_DIR ${CMAKE_CURRENT_LIST_DIR}/../../../external/INSLIB)

add_library(inslib STATIC
    ${INSLIB_DIR}/src/ins.c
    ${INSLIB_DIR}/src/ahrs.c
    ${INSLIB_DIR}/src/baro_alt.c
    ${INSLIB_DIR}/src/nav_suite.c
    ${INSLIB_DIR}/src/geodetic_toolbox.c
    ${INSLIB_DIR}/src/magnetic_model.c
    ${INSLIB_DIR}/src/log.c
    ${INSLIB_DIR}/KFCore/c/linalg.c
    ${INSLIB_DIR}/KFCore/c/kalman_udu.c
    ${INSLIB_DIR}/KFCore/c/miniblas.c
)

target_include_directories(inslib PUBLIC
    ${INSLIB_DIR}/src
    ${INSLIB_DIR}/KFCore/c
)

# Same limits as INSLIB's own Makefile: the 18-state filter with 24 noise
# columns. KFCore sizes its stack scratch matrices from them.
target_compile_definitions(inslib PUBLIC
    KALMAN_MAX_STATE_SIZE=18
    KALMAN_MAX_NOISE_SIZE=24
)

target_compile_options(inslib PRIVATE -O2)

# For the SDK's float and printf implementations at link time.
target_link_libraries(inslib PRIVATE pico_stdlib)
