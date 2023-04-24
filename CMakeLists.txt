cmake_minimum_required(VERSION 3.13)

# initialize the SDK directly
include(/home/jack/pico/pico-sdk/pico_sdk_init.cmake)

# cmake -DPICO_BOARD=pico .. && make
project(pico-robotics)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(pico-robotics
    main.c
    KitronikPicoRobotics.c)

target_include_directories(pico-robotics PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/..)

target_link_libraries(pico-robotics
    pico_stdlib
    hardware_gpio
    hardware_i2c
    hardware_pwm)

pico_add_extra_outputs(pico-robotics)
