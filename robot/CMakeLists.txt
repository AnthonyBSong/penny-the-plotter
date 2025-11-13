# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================

# Use Pico W (has WiFi + lwIP)
set(PICO_BOARD pico_w CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.13)

# Pull in the Pico SDK (VS Code style)
include(pico_sdk_import.cmake)

project(robot C CXX ASM)

pico_sdk_init()

add_executable(robot)

target_sources(robot PRIVATE
    robot.c
)

target_include_directories(robot PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(robot
    pico_cyw43_arch_lwip_threadsafe_background  # WiFi + lwIP
    pico_stdlib
    pico_bootsel_via_double_reset
    hardware_sync
    pico_sync
)

pico_add_extra_outputs(robot)