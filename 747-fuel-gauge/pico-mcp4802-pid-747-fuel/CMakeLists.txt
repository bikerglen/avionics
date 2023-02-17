cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(fuel747 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(fuel747)

pico_enable_stdio_usb(fuel747 0)
pico_enable_stdio_uart(fuel747 1)

target_sources(fuel747 PRIVATE main.cpp pwl.cpp)

target_include_directories(fuel747 PUBLIC
        ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(fuel747 PRIVATE pico_stdlib pico_multicore pico_unique_id pico_unique_id hardware_spi hardware_adc)
pico_add_extra_outputs(fuel747)
