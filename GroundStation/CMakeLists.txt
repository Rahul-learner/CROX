cmake_minimum_required(VERSION 3.13...3.27)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# initialize the SDK based on PICO_SDK_PATH
include(../pico_sdk_import.cmake)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(GroundStation C CXX ASM)

pico_sdk_init()

add_executable(GroundStation
    main.cpp
)

pico_set_program_name(GroundStation "GroundStation")
pico_set_program_version(GroundStation "0.1")

# Enable output over USB
pico_enable_stdio_uart(GroundStation 0)
pico_enable_stdio_usb(GroundStation 1)

target_link_libraries(GroundStation
    pico_stdlib
    hardware_spi
)

target_include_directories(GroundStation PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ../include
    ../config
    ../src
)

pico_add_extra_outputs(GroundStation)
