cmake_minimum_required(VERSION 3.13)

set(PICO_SDK_PATH "/usr/share/pico-sdk")
include(pico_sdk_import.cmake)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

project(AP_PHYS_1_EOY C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

add_executable(AP_PHYS_1_EOY
        src/main.c
        src/flight_math.c
)

target_link_libraries(AP_PHYS_1_EOY
        pico_stdlib
        hardware_i2c
)

pico_enable_stdio_usb(AP_PHYS_1_EOY 1)
pico_enable_stdio_uart(AP_PHYS_1_EOY 0)

pico_add_extra_outputs(AP_PHYS_1_EOY)