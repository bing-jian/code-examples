cmake_minimum_required(VERSION 2.8.12)
project(opencv_pybind11_example)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

#find_package (OpenCV REQUIRED)
#include_directories(${OpenCV_INCLUDE_DIRS})
#message(STATUS "OpenCV_INCLUDE_DIRS => ${OpenCV_INCLUDE_DIRS}")

add_subdirectory(python-binding)
