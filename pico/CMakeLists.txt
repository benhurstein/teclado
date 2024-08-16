cmake_minimum_required(VERSION 3.13...3.30)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
include(pico_sdk_import.cmake)
project(teclado_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()
add_executable(teclado teclado.c usb_descriptors.c)

# Make sure TinyUSB can find tusb_config.h
target_include_directories(teclado PUBLIC
        ${CMAKE_CURRENT_LIST_DIR})
        
pico_enable_stdio_usb(teclado 1)
pico_enable_stdio_uart(teclado 0)
pico_add_extra_outputs(teclado)
target_link_libraries(teclado pico_stdlib hardware_adc tinyusb_device tinyusb_board hardware_pio)

file(MAKE_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/generated)

# generate the header file into the source tree as it is included in the RP2040 datasheet
pico_generate_pio_header(teclado ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/generated)
