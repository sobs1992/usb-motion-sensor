cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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
set(PICO_BOARD pico_w CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(usb_motion_sensor C CXX ASM)

pico_sdk_init()

# ---- Build options -----------------------------------------------------------
# GPIO wired to the ICM-20948 INT pin (-1 = not wired, poll INT_STATUS over I2C).
set(IMU_INT_GPIO -1 CACHE STRING "GPIO connected to ICM-20948 INT, -1 to poll")
set(IMU_I2C_SDA 4 CACHE STRING "I2C0 SDA GPIO")
set(IMU_I2C_SCL 5 CACHE STRING "I2C0 SCL GPIO")

add_executable(usb_motion_sensor
    src/main.c
    src/icm20948.c
    src/protocol.c
    usb/usb_descriptors.c
    bsp/rp2040/bsp_rp2040.c
)

target_include_directories(usb_motion_sensor PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
    ${CMAKE_CURRENT_LIST_DIR}/bsp
    ${CMAKE_CURRENT_LIST_DIR}/usb      # tusb_config.h
)

target_compile_definitions(usb_motion_sensor PRIVATE
    IMU_INT_GPIO=${IMU_INT_GPIO}
    IMU_I2C_SDA=${IMU_I2C_SDA}
    IMU_I2C_SCL=${IMU_I2C_SCL}
)

target_compile_options(usb_motion_sensor PRIVATE -Wall -Wextra -Wno-unused-parameter)

target_link_libraries(usb_motion_sensor PRIVATE
    pico_stdlib
    pico_unique_id
    pico_bootrom
    hardware_i2c
    tinyusb_device
)

pico_set_program_name(usb_motion_sensor "usb_motion_sensor")
pico_set_program_version(usb_motion_sensor "0.1")

# No stdio at all: USB is owned by our own TinyUSB CDC stream.
pico_enable_stdio_uart(usb_motion_sensor 0)
pico_enable_stdio_usb(usb_motion_sensor 0)

pico_add_extra_outputs(usb_motion_sensor)
