cmake_minimum_required(VERSION 2.8.12)
project(opencv_pybind11_example)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Find OpenCV -DOpenCV_DIR=...
find_package (OpenCV REQUIRED)
message(STATUS "OpenCV_INCLUDE_DIRS => ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_LIBS : ${OpenCV_LIBS}")
include_directories(${OpenCV_INCLUDE_DIRS})

add_library(depth_utils depth_utils.cc)
target_link_libraries(depth_utils ${OpenCV_LIBS})
add_subdirectory(python-binding)
