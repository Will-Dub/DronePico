cmake_minimum_required(VERSION 3.12)

#include build fonction from pico sdk
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
include($ENV{PICO_SDK_PATH}/tools/CMakeLists.txt)

#set name of projet (as PROJECT_NAME) and c/c++ standard
project(dronePico C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

#Tell executable file
add_executable(${PROJECT_NAME}
    main.cpp
)

target_sources(${PROJECT_NAME} PRIVATE
    i2c.cpp
    i2c.h
    mpu6050.cpp
    mpu6050.h
    qmc5883l.cpp
    qmc5883l.h
    TinyGPS.cpp
    TinyGPS.h
    message.cpp
    uart.cpp
    uart.h
)

#Link to lib
target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    hardware_i2c
    hardware_uart
    hardware_sync
    pico_multicore
)

#create map/bin/hex/uf2 file
pico_add_extra_outputs(${PROJECT_NAME})

# USB output, disable uart output
pico_enable_stdio_usb(${PROJECT_NAME} 0)
pico_enable_stdio_uart(${PROJECT_NAME} 0)
