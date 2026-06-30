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

if(DEFINED MOTOR_UART_ID)
    set(DIAGNOSTIC_UART_ID_DEFAULT ${MOTOR_UART_ID})
else()
    set(DIAGNOSTIC_UART_ID_DEFAULT 1)
endif()
if(DEFINED MOTOR_UART_BAUDRATE)
    set(DIAGNOSTIC_UART_BAUDRATE_DEFAULT ${MOTOR_UART_BAUDRATE})
else()
    set(DIAGNOSTIC_UART_BAUDRATE_DEFAULT 115200)
endif()
if(DEFINED MOTOR_UART_TX_GPIO)
    set(DIAGNOSTIC_UART_TX_GPIO_DEFAULT ${MOTOR_UART_TX_GPIO})
else()
    set(DIAGNOSTIC_UART_TX_GPIO_DEFAULT 6)
endif()
if(DEFINED MOTOR_UART_RX_GPIO)
    set(DIAGNOSTIC_UART_RX_GPIO_DEFAULT ${MOTOR_UART_RX_GPIO})
else()
    set(DIAGNOSTIC_UART_RX_GPIO_DEFAULT 7)
endif()

# ============================================================
# WolveZ 2026 XIAO RP2350 hardware profile
# ------------------------------------------------------------
# These defaults are for the 2026 motor-driver board.
# They can still be overridden by -D... CMake arguments.
#
# UART:
#   Raspberry Pi GPIO12 / TXD5 -> XIAO RX GPIO1
#   Raspberry Pi GPIO13 / RXD5 <- XIAO TX GPIO0
#
# Motor direct GPIO diagnostic result:
#   GPIO11 -> motor1 clockwise
#   GPIO20 -> motor1 counter-clockwise
#   GPIO12 -> motor2 counter-clockwise
#   GPIO21 -> motor2 clockwise
# ============================================================
set(MOTOR_UART_ID 0 CACHE STRING "UART id for Raspberry Pi link")
set(MOTOR_UART_BAUDRATE 115200 CACHE STRING "UART baudrate for Raspberry Pi link")
set(MOTOR_UART_TX_GPIO 0 CACHE STRING "XIAO UART TX GPIO")
set(MOTOR_UART_RX_GPIO 1 CACHE STRING "XIAO UART RX GPIO")

set(MOTOR_PWM0_GPIO 20 CACHE STRING "Motor PWM0 GPIO")
set(MOTOR_PWM1_GPIO 21 CACHE STRING "Motor PWM1 GPIO")
set(MOTOR_PWM2_GPIO 12 CACHE STRING "Motor PWM2 GPIO")
set(MOTOR_PWM3_GPIO 11 CACHE STRING "Motor PWM3 GPIO")

set(MOTOR_SERVO_GPIO 18 CACHE STRING "Servo GPIO")
set(MOTOR_QENC0_GPIO 26 CACHE STRING "QENC0 GPIO")
set(MOTOR_QENC1_GPIO 28 CACHE STRING "QENC1 GPIO")
set(DIAGNOSTIC_UART_ID ${DIAGNOSTIC_UART_ID_DEFAULT} CACHE STRING "Diagnostic UART instance id: 0 for uart0, 1 for uart1")
set(DIAGNOSTIC_UART_BAUDRATE ${DIAGNOSTIC_UART_BAUDRATE_DEFAULT} CACHE STRING "Diagnostic UART baudrate")
set(DIAGNOSTIC_UART_TX_GPIO ${DIAGNOSTIC_UART_TX_GPIO_DEFAULT} CACHE STRING "Diagnostic UART TX GPIO")
set(DIAGNOSTIC_UART_RX_GPIO ${DIAGNOSTIC_UART_RX_GPIO_DEFAULT} CACHE STRING "Diagnostic UART RX GPIO")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

if (PICO_SDK_VERSION_STRING VERSION_LESS "2.0.0")
  message(FATAL_ERROR "Raspberry Pi Pico SDK version 2.0.0 (or later) required for XIAO RP2350. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

project(rpico-motor-driver C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

message(STATUS "rpico-motor-driver PICO_BOARD=${PICO_BOARD}")

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
pico_set_program_version(rpico-motor-driver "0.3-xiao-rp2350-safe-diag")

# Generate PIO header
pico_generate_pio_header(rpico-motor-driver ${CMAKE_CURRENT_LIST_DIR}/encoder.pio)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(rpico-motor-driver 1)
pico_enable_stdio_usb(rpico-motor-driver 0)

target_compile_definitions(rpico-motor-driver PRIVATE
        MOTOR_FIRMWARE_VERSION="0.3-xiao-rp2350-safe-diag"
        MOTOR_PICO_BOARD="${PICO_BOARD}"
        MOTOR_UART_ID=${MOTOR_UART_ID}
        MOTOR_UART_BAUDRATE=${MOTOR_UART_BAUDRATE}
        MOTOR_UART_TX_GPIO=${MOTOR_UART_TX_GPIO}
        MOTOR_UART_RX_GPIO=${MOTOR_UART_RX_GPIO}
        MOTOR_SERVO_GPIO=${MOTOR_SERVO_GPIO}
        MOTOR_PWM0_GPIO=${MOTOR_PWM0_GPIO}
        MOTOR_PWM1_GPIO=${MOTOR_PWM1_GPIO}
        MOTOR_PWM2_GPIO=${MOTOR_PWM2_GPIO}
        MOTOR_PWM3_GPIO=${MOTOR_PWM3_GPIO}
        MOTOR_QENC0_GPIO=${MOTOR_QENC0_GPIO}
        MOTOR_QENC1_GPIO=${MOTOR_QENC1_GPIO}
        )

message(STATUS "rpico-motor-driver MOTOR_UART_ID=${MOTOR_UART_ID}")
message(STATUS "rpico-motor-driver MOTOR_UART_BAUDRATE=${MOTOR_UART_BAUDRATE}")
message(STATUS "rpico-motor-driver MOTOR_UART_TX_GPIO=${MOTOR_UART_TX_GPIO}")
message(STATUS "rpico-motor-driver MOTOR_UART_RX_GPIO=${MOTOR_UART_RX_GPIO}")
message(STATUS "rpico-motor-driver MOTOR_SERVO_GPIO=${MOTOR_SERVO_GPIO}")
message(STATUS "rpico-motor-driver MOTOR_PWM=${MOTOR_PWM0_GPIO},${MOTOR_PWM1_GPIO},${MOTOR_PWM2_GPIO},${MOTOR_PWM3_GPIO}")
message(STATUS "rpico-motor-driver MOTOR_QENC=${MOTOR_QENC0_GPIO},${MOTOR_QENC1_GPIO}")

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

add_executable(rpico-motor-driver-diagnostic-uart-d6d7
        diagnostic_uart_d6d7.cpp
        )

pico_set_program_name(rpico-motor-driver-diagnostic-uart-d6d7 "rpico-motor-driver-diagnostic-uart-d6d7")
pico_set_program_version(rpico-motor-driver-diagnostic-uart-d6d7 "0.1")

target_compile_definitions(rpico-motor-driver-diagnostic-uart-d6d7 PRIVATE
        MOTOR_UART_ID=${DIAGNOSTIC_UART_ID}
        MOTOR_UART_BAUDRATE=${DIAGNOSTIC_UART_BAUDRATE}
        MOTOR_UART_TX_GPIO=${DIAGNOSTIC_UART_TX_GPIO}
        MOTOR_UART_RX_GPIO=${DIAGNOSTIC_UART_RX_GPIO}
        )

message(STATUS "rpico-motor-driver-diagnostic-uart-d6d7 MOTOR_UART_ID=${DIAGNOSTIC_UART_ID}")
message(STATUS "rpico-motor-driver-diagnostic-uart-d6d7 MOTOR_UART_BAUDRATE=${DIAGNOSTIC_UART_BAUDRATE}")
message(STATUS "rpico-motor-driver-diagnostic-uart-d6d7 MOTOR_UART_TX_GPIO=${DIAGNOSTIC_UART_TX_GPIO}")
message(STATUS "rpico-motor-driver-diagnostic-uart-d6d7 MOTOR_UART_RX_GPIO=${DIAGNOSTIC_UART_RX_GPIO}")

target_link_libraries(rpico-motor-driver-diagnostic-uart-d6d7
        pico_stdlib
        pico_stdio_uart
        hardware_uart
        )

pico_enable_stdio_uart(rpico-motor-driver-diagnostic-uart-d6d7 1)
pico_enable_stdio_usb(rpico-motor-driver-diagnostic-uart-d6d7 0)

pico_add_extra_outputs(rpico-motor-driver-diagnostic-uart-d6d7)
