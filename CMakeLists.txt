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
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(pico_two_program C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# This is where you add new source files
add_executable(pico_one_program
    src/main_pico_one.cpp
    src/pico-one/io_core.cpp
    src/pico-one/io_manager.cpp
    libs/imported/pico_servo.c
    libs/imported/QuadratureEncoder.cpp
    libs/imported/mp92plus.c
    libs/imported/zsl/matrices.c
    libs/imported/zsl/probability.c
    libs/imported/zsl/vectors.c
    libs/imported/zsl/statistics.c
    libs/imported/zsl/zsl.c
    libs/LiDAR.cpp
    libs/CoordProcess.hpp
)

pico_set_program_name(pico_one_program "pico_one_program")
pico_set_program_version(pico_one_program "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(pico_one_program 1)
pico_enable_stdio_usb(pico_one_program 1)
pico_generate_pio_header(pico_one_program ${CMAKE_CURRENT_LIST_DIR}/libs/imported/quadrature_encoder.pio)

# Add any pico libraries you use here to link them
target_link_libraries(pico_one_program
    pico_stdlib
    pico_multicore
    hardware_pwm
    hardware_pio
    hardware_spi
    hardware_i2c
    pico_i2c_slave
)

# Add the standard include files to the build
target_include_directories(pico_one_program 
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/src
        ${CMAKE_CURRENT_LIST_DIR}/libs
        ${CMAKE_CURRENT_LIST_DIR}/libs/imported
)

pico_add_extra_outputs(pico_one_program)


project(pico_two_program C CXX ASM)

# This is where you add new source files
add_executable(pico_two_program
    src/main_pico_two.cpp
    libs/Motor.cpp
)

pico_set_program_name(pico_two_program "pico_two_program")
pico_set_program_version(pico_two_program "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(pico_two_program 1)
pico_enable_stdio_usb(pico_two_program 1)

# Add any pico libraries you use here to link them
target_link_libraries(pico_two_program
    pico_stdlib
    pico_multicore
    hardware_pwm
    hardware_pio
    hardware_spi
    hardware_i2c
    pico_i2c_slave
)

# Add the standard include files to the build
target_include_directories(pico_two_program 
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/src
        ${CMAKE_CURRENT_LIST_DIR}/libs
        ${CMAKE_CURRENT_LIST_DIR}/libs/imported
        ${CMAKE_CURRENT_LIST_DIR}/src/pico-two
)

pico_add_extra_outputs(pico_two_program)




project(motor_driver_test C CXX ASM)

# This is where you add new source files
add_executable(motor_driver_test
    test/motor-class.cpp
    libs/Motor.cpp
)

pico_set_program_name(motor_driver_test "pico_two_program")
pico_set_program_version(motor_driver_test "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(motor_driver_test 1)
pico_enable_stdio_usb(motor_driver_test 1)

# Add any pico libraries you use here to link them
target_link_libraries(motor_driver_test
    pico_stdlib
    hardware_pwm
)

# Add the standard include files to the build
target_include_directories(motor_driver_test 
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/libs
)

pico_add_extra_outputs(motor_driver_test)


