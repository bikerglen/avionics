cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(gearflaps C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(gearflaps)

pico_enable_stdio_usb(gearflaps 0)
pico_enable_stdio_uart(gearflaps 1)

target_sources(gearflaps PRIVATE main.cpp)

target_include_directories(gearflaps PUBLIC
        ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(gearflaps PRIVATE pico_stdlib pico_unique_id pico_unique_id hardware_spi)
pico_add_extra_outputs(gearflaps)
