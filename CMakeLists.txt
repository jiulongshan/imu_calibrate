cmake_minimum_required(VERSION 3.5)

project("imu_calibration_lib" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# add_definitions("-Wall -g")

#include dirs
include_directories(
    ./include
)

#link dirs
link_directories(
)

#link libs
link_libraries(
    -lpthread
)

add_library(imu_calibration SHARED
    ./src/imu_protocol.cpp
    ./src/serialport.cpp
    ./src/io_interface.cpp
    ./src/imu_data_handle.cpp
    ./src/utils.cpp
)
