# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(Arm C CXX ASM)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(Arm Arm.cpp )

pico_set_program_name(Arm "Arm")
pico_set_program_version(Arm "0.1")

# Generate PIO header
pico_generate_pio_header(Arm ${CMAKE_CURRENT_LIST_DIR}/blink.pio)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(Arm 1)
pico_enable_stdio_usb(Arm 1)

# Add the standard library to the build
target_link_libraries(Arm
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(Arm PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${PICO_SDK_PATH}/lib/lwip/src/include
)

# Add any user requested libraries
target_link_libraries(Arm 
        hardware_spi
        hardware_i2c
        hardware_dma
        hardware_pio
        hardware_interp
        hardware_timer
        hardware_watchdog
        hardware_clocks
        hardware_pwm
        hardware_uart
        pico_multicore
        )

pico_add_extra_outputs(Arm)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${PICO_SDK_PATH}/lib/lwip/src/include
)

target_link_libraries(${PROJECT_NAME}
    pico_cyw43_arch_lwip_poll
    pico_lwip_nosys
)

find_library(MMAL_LIBRARIES mmal)
find_library(MMAL_UTIL_LIBRARIES mmal_util)
find_library(MMAL_CORE_LIBRARIES mmal_core)