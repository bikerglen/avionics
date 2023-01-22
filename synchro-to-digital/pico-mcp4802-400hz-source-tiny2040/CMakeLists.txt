cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(sin400 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PICO_BOARD pimoroni_tiny2040)
pico_sdk_init()

add_executable(sin400)

pico_enable_stdio_usb(sin400 0)
pico_enable_stdio_uart(sin400 1)

target_sources(sin400 PRIVATE main.cpp)

target_include_directories(sin400 PUBLIC
        ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(sin400 PRIVATE pico_stdlib pico_multicore pico_unique_id pico_unique_id hardware_spi hardware_adc)

pico_add_extra_outputs(sin400)
