cmake_minimum_required(VERSION 3.13)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 1.5.1)
set(toolchainVersion 13_2_Rel1)
set(picotoolVersion 1.5.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.4.0")
  message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.4.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

project(rpico-motor-driver C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(rpico-motor-driver
          rpico-motor-driver.cpp
          ./lib/rpico-pwm/pwm.cpp
          ./lib/rpico-encoder-plus/qenc.cpp
          ./lib/rpico-motor/motor.cpp
          ./lib/generic-pid/pid.cpp
          ./lib/rpico-servo/servo.cpp
          )

pico_set_program_name(rpico-motor-driver "rpico-motor-driver")
pico_set_program_version(rpico-motor-driver "0.1")

# Generate PIO header
pico_generate_pio_header(rpico-motor-driver ${CMAKE_CURRENT_LIST_DIR}/encoder.pio)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(rpico-motor-driver 1)
pico_enable_stdio_usb(rpico-motor-driver 0)

# Add the standard library to the build
target_link_libraries(rpico-motor-driver
        pico_stdlib
        hardware_pwm
        )

# Add the standard include files to the build
target_include_directories(rpico-motor-driver PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
)

# Add any user requested libraries
target_link_libraries(rpico-motor-driver 
        hardware_pio
        )

pico_add_extra_outputs(rpico-motor-driver)

