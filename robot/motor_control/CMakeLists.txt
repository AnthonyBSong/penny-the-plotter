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

# cmake version
cmake_minimum_required(VERSION 3.13)

# include the sdk.cmake file
include(pico_sdk_import.cmake)

# give the project a name (anything you want)
project(Motor_Ctrl C CXX ASM)

# initialize the sdk
pico_sdk_init()

add_executable(Motor_Ctrl
    motor_ctrl.c             
    vga16_graphics_v2.c    
    mpu6050.c              
)

# must match with pio filename and executable name from above
pico_generate_pio_header(Motor_Ctrl ${CMAKE_CURRENT_LIST_DIR}/hsync.pio)
pico_generate_pio_header(Motor_Ctrl ${CMAKE_CURRENT_LIST_DIR}/vsync.pio)
pico_generate_pio_header(Motor_Ctrl ${CMAKE_CURRENT_LIST_DIR}/rgb.pio)

# Add pico_multicore which is required for multicore functionality
target_link_libraries(Motor_Ctrl
    pico_stdlib
    pico_multicore
    pico_bootsel_via_double_reset
    hardware_pwm
    hardware_irq
    hardware_i2c
    hardware_dma
    hardware_pio
    hardware_clocks
    hardware_adc 
    
)

pico_enable_stdio_usb(Motor_Ctrl 0)
pico_enable_stdio_uart(Motor_Ctrl 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(Motor_Ctrl)
