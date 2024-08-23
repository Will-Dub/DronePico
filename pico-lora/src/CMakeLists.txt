cmake_minimum_required(VERSION 3.12)

# Add library sources
add_library(LoRa_pico_lib
    LoRa-RP2040.cpp
    Print.cpp
)

# Specify the include directories for the second project
target_include_directories(LoRa_pico_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Link the Pico SDK and other libraries required by LoRa_pico_lib
target_link_libraries(LoRa_pico_lib
    pico_stdlib
    hardware_spi
    hardware_interp
    LoRa_print
)

# Optionally, if you need a separate library for Print, you can define it here
add_library(LoRa_print
    Print.cpp
    Print.h
)

# Link the Pico SDK and other libraries required by LoRa_print
target_link_libraries(LoRa_print pico_stdlib)