cmake_minimum_required(VERSION 3.13)

# Pull in Pico SDK (expects PICO_SDK_PATH env var or local pico_sdk_import.cmake)
include(pico_sdk_import.cmake)

project(vcv_bridge C CXX ASM)
set(CMAKE_C_STANDARD   11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(vcv_bridge
    vcv_bridge.cpp
    usb_descriptors.c
)

# ComputerCard.h must be on the include path.
# Point COMPUTERCARD_DIR at the checked-out ComputerCard repository.
# Override with: cmake -DCOMPUTERCARD_DIR=../path/to/ComputerCard ..
if(NOT DEFINED COMPUTERCARD_DIR)
    set(COMPUTERCARD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../deps/ComputerCard")
endif()
target_include_directories(vcv_bridge PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}     # tusb_config.h, BridgeProtocol.hpp
    ${COMPUTERCARD_DIR}             # ComputerCard.h
)

target_link_libraries(vcv_bridge
    pico_stdlib
    pico_multicore
    hardware_pwm
    hardware_adc
    hardware_i2s
    hardware_dma
    hardware_irq
    tinyusb_device
    tinyusb_board
)

# Generate UF2 + other outputs
pico_add_extra_outputs(vcv_bridge)

# Disable stdio over USB (we use raw CDC — stdio_usb would conflict)
pico_enable_stdio_usb(vcv_bridge 0)
pico_enable_stdio_uart(vcv_bridge 0)

# Flash at 250 MHz for headroom (comment out if unstable)
# target_compile_definitions(vcv_bridge PRIVATE SYS_CLK_KHZ=250000)
