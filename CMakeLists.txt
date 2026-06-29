cmake_minimum_required(VERSION 3.13)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.0.0)
set(toolchainVersion 13_2_Rel1)
set(picotoolVersion 1.5.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD seeed_xiao_rp2350 CACHE STRING "Board type")
set(MOTOR_UART_INSTANCE_INDEX 1 CACHE STRING "UART instance index: 0 for uart0, 1 for uart1")
set(MOTOR_UART_TX_GPIO 20 CACHE STRING "XIAO RP2350 GPIO connected to board PICO_TX")
set(MOTOR_UART_RX_GPIO 21 CACHE STRING "XIAO RP2350 GPIO connected to board PICO_RX")
set(MOTOR_UART_BAUDRATE 115200 CACHE STRING "Motor command UART baudrate")
option(MOTOR_BUILD_DIAGNOSTIC "Build the UART/GPIO diagnostic firmware with no motor or servo control" ON)
set(MOTOR_DIAGNOSTIC_FIRMWARE_ID "uart-gpio-diagnostic-no-motor-tx${MOTOR_UART_TX_GPIO}-rx${MOTOR_UART_RX_GPIO}" CACHE STRING "Diagnostic firmware ID")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

if (PICO_SDK_VERSION_STRING VERSION_LESS "2.0.0")
  message(FATAL_ERROR "Raspberry Pi Pico SDK version 2.0.0 (or later) required for XIAO RP2350. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

project(rpico-motor-driver C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

message(STATUS "rpico-motor-driver PICO_BOARD=${PICO_BOARD}")
message(STATUS "rpico-motor-driver UART_INSTANCE=uart${MOTOR_UART_INSTANCE_INDEX}")
message(STATUS "rpico-motor-driver UART_TX_GPIO=${MOTOR_UART_TX_GPIO}")
message(STATUS "rpico-motor-driver UART_RX_GPIO=${MOTOR_UART_RX_GPIO}")
message(STATUS "rpico-motor-driver UART_BAUDRATE=${MOTOR_UART_BAUDRATE}")

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
pico_set_program_version(rpico-motor-driver "0.2-xiao-rp2350-uart-debug")

# Generate PIO header
pico_generate_pio_header(rpico-motor-driver ${CMAKE_CURRENT_LIST_DIR}/encoder.pio)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(rpico-motor-driver 1)
pico_enable_stdio_usb(rpico-motor-driver 0)

target_compile_definitions(rpico-motor-driver PRIVATE
        MOTOR_FIRMWARE_VERSION="0.2-xiao-rp2350-uart-debug"
        MOTOR_PICO_BOARD="${PICO_BOARD}"
        MOTOR_UART_INSTANCE_INDEX=${MOTOR_UART_INSTANCE_INDEX}
        MOTOR_UART_TX_GPIO=${MOTOR_UART_TX_GPIO}
        MOTOR_UART_RX_GPIO=${MOTOR_UART_RX_GPIO}
        MOTOR_UART_BAUDRATE=${MOTOR_UART_BAUDRATE}
        )

# Add the standard library to the build
target_link_libraries(rpico-motor-driver
        pico_stdlib
        pico_stdio_uart
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

if (MOTOR_BUILD_DIAGNOSTIC)
add_executable(rpico-motor-driver-diagnostic
          diagnostic_uart_gpio.cpp
          )

pico_set_program_name(rpico-motor-driver-diagnostic "rpico-motor-driver-diagnostic")
pico_set_program_version(rpico-motor-driver-diagnostic "${MOTOR_DIAGNOSTIC_FIRMWARE_ID}")

pico_enable_stdio_uart(rpico-motor-driver-diagnostic 1)
pico_enable_stdio_usb(rpico-motor-driver-diagnostic 1)

target_compile_definitions(rpico-motor-driver-diagnostic PRIVATE
        MOTOR_DIAGNOSTIC_FIRMWARE_ID="${MOTOR_DIAGNOSTIC_FIRMWARE_ID}"
        MOTOR_PICO_BOARD="${PICO_BOARD}"
        MOTOR_UART_INSTANCE_INDEX=${MOTOR_UART_INSTANCE_INDEX}
        MOTOR_UART_TX_GPIO=${MOTOR_UART_TX_GPIO}
        MOTOR_UART_RX_GPIO=${MOTOR_UART_RX_GPIO}
        MOTOR_UART_BAUDRATE=${MOTOR_UART_BAUDRATE}
        )

target_link_libraries(rpico-motor-driver-diagnostic
        pico_stdlib
        pico_stdio_uart
        pico_stdio_usb
        )

target_include_directories(rpico-motor-driver-diagnostic PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
)

pico_add_extra_outputs(rpico-motor-driver-diagnostic)
endif()
