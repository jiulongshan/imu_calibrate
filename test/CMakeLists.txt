# CMakeList.txt: DeviceApi 的 CMake 项目，在此处包括源代码并定义
# 项目特定的逻辑。
#
cmake_minimum_required (VERSION 3.5)

project ("test" CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions("-g")

#find_package(OpenCV REQUIRED)

# 头文件目录
include_directories(
    ../include/
    ../src/
)

link_directories(
    ../build/
)

link_libraries(
    -limu_calibration
)

add_executable (imu_test 
    test.cpp
)



