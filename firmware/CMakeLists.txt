cmake_minimum_required(VERSION 3.13)

if(DEFINED PICO_SDK_PATH)
    set(ENV{PICO_SDK_PATH} "${PICO_SDK_PATH}")
elseif(DEFINED ENV{PICO_SDK_PATH})
    set(PICO_SDK_PATH "$ENV{PICO_SDK_PATH}")
else()
    message(FATAL_ERROR "PICO_SDK_PATH is not set. Install pico-sdk and export PICO_SDK_PATH=/path/to/pico-sdk or pass -DPICO_SDK_PATH=/path/to/pico-sdk")
endif()

include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(rp2040_swim C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(rp2040_swim
    src/main.c
    src/usb_protocol.c
    src/swim_pio_waveform.c
    src/swim_pio_rx.c
    src/swim_phy.c
    src/swim_link.c
    src/stm8_dm.c
    src/stm8_flash.c
    src/target_db.c
)

pico_generate_pio_header(rp2040_swim ${CMAKE_CURRENT_LIST_DIR}/pio/swim_waveform.pio)
pico_generate_pio_header(rp2040_swim ${CMAKE_CURRENT_LIST_DIR}/pio/swim_rx_width.pio)

target_include_directories(rp2040_swim PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
)

target_link_libraries(rp2040_swim
    pico_stdlib
    hardware_clocks
    hardware_dma
    hardware_gpio
    hardware_pio
    hardware_timer
)

pico_enable_stdio_usb(rp2040_swim 1)
pico_enable_stdio_uart(rp2040_swim 0)
pico_add_extra_outputs(rp2040_swim)
