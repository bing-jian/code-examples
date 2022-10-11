cmake_minimum_required(VERSION 2.8.12)
project(eigen_pybind11_example)

set(CMAKE_CXX_STANDARD 14)
set(cmake_position_independent_code on)

find_package (Eigen3 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(/usr/local/include)

add_library(core math_utils.cc funcs.cc geometry_utils.cc distance_transform_utils.cc)
add_subdirectory(python-binding)
