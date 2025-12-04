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
set(PICO_BOARD pico CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)

project(Motor_Ctrl C CXX ASM)

pico_sdk_init()

add_executable(Motor_Ctrl
    motor_ctrl.c
)

target_link_libraries(Motor_Ctrl
    pico_stdlib
    hardware_pwm
)

# Choose one or both depending on how you read printf:
pico_enable_stdio_usb(Motor_Ctrl 0)   # USB CDC off
pico_enable_stdio_uart(Motor_Ctrl 1)  # UART on

pico_add_extra_outputs(Motor_Ctrl)
