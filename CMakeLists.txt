cmake_minimum_required(VERSION 3.13...3.27)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)
#Overclock Pico
set(PICO_DEFAULT_CPU_FREQ_MHZ 250)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(CROX C CXX ASM)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(CROX
    main.cpp
    config/config.h
    config/config.cpp
    src/core/hardware_setup.cpp
    src/core/flight_tasks.cpp
    src/core/comm_tasks.cpp
    src/core/sitl.cpp
    src/EKF.cpp
    include/EKF.h
    src/readIMU.cpp
    include/readIMU.h
    src/readPWM.cpp
    include/readPWM.h
    src/writePWM.cpp
    include/writePWM.h
    src/PID.cpp
    include/PID.h
    src/PwmIn.pio
    src/buzzer.cpp
    include/buzzer.h
    src/nrf24_radio.cpp
    include/nrf24_radio.h
    src/msp_protocol.cpp
    include/msp_protocol.h
    src/telemetry.cpp
    include/telemetry.h
    src/calibration.cpp
    include/calibration.h
    src/blackbox.cpp
    include/blackbox.h
)

pico_generate_pio_header(CROX ${CMAKE_CURRENT_LIST_DIR}/src/PwmIn.pio)

pico_set_program_name(CROX "CROX")
pico_set_program_version(CROX "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(CROX 0)
pico_enable_stdio_usb(CROX 1)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(CROX
        pico_stdlib
        hardware_i2c
        hardware_clocks
        hardware_pio
        hardware_pwm
        hardware_uart
        pico_multicore
        hardware_spi
        hardware_flash
        hardware_sync
        m)

# Add the standard include files to the build
target_include_directories(CROX PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/include
        ${CMAKE_CURRENT_LIST_DIR}/config
        )
# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(CROX)

