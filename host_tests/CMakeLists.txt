cmake_minimum_required(VERSION 3.16)
project(rpico_motor_driver_host_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

enable_testing()

add_library(firmware_logic STATIC
    ../firmware_logic.cpp
)
target_include_directories(firmware_logic PUBLIC ..)

add_executable(firmware_logic_test
    firmware_logic_test.cpp
)
target_link_libraries(firmware_logic_test PRIVATE firmware_logic)
add_test(NAME firmware_logic_test COMMAND firmware_logic_test)

add_executable(sync_profile_test
    ../sync_profile_test.cpp
)
target_link_libraries(sync_profile_test PRIVATE firmware_logic)
add_test(NAME sync_profile_test COMMAND sync_profile_test)
