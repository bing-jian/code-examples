# Notes:
# 1) Install pybind11 using conda
#    conda install pybind11
# 2) Config Python version accordingly when using CMake

find_package(pybind11 REQUIRED CONFIG)
include_directories(${pybind11_INCLUDE_DIR})
pybind11_add_module(wrapper pybind11_interface.cc)
target_link_libraries(wrapper PRIVATE core)
