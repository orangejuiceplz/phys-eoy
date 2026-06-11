cmake_minimum_required(VERSION 3.13)

set(PICO_SDK_PATH "/usr/share/pico-sdk")
set(PICO_BOARD pico2)
set(PICO_PLATFORM rp2350-arm-s)

include(pico_sdk_import.cmake)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

project(AP_PHYS_1_EOY C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

file(GLOB VL53L4CX_SOURCES
        lib/vl53l4cx/core/*.c
        lib/vl53l4cx/platform/*.c
)

add_executable(AP_PHYS_1_EOY
        src/main.c
        src/flight_math.c
        src/state_machine.c
        src/3.3v/bmp280.c
        src/3.3v/mpu6050.c
        src/3.3v/vl53l4cx.c
        src/5v/servo.c
        src/5v/esc.c
        src/fusion.c
        src/flash_state.c
        ${VL53L4CX_SOURCES}
)

target_include_directories(AP_PHYS_1_EOY PRIVATE
        ${CMAKE_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/lib/vl53l4cx/core
        ${CMAKE_SOURCE_DIR}/lib/vl53l4cx/platform
)

target_link_libraries(AP_PHYS_1_EOY
        pico_stdlib
        hardware_i2c
        hardware_pwm
        hardware_flash
        hardware_sync
)

pico_enable_stdio_usb(AP_PHYS_1_EOY 1)
pico_enable_stdio_uart(AP_PHYS_1_EOY 0)

pico_add_extra_outputs(AP_PHYS_1_EOY)