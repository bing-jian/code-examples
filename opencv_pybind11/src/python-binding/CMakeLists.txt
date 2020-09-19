# Important notes:
# 1) Install pybind11 using conda
#    conda install pybind11
# 2) Config Python version accordingly when using CMake
#    make sure the PYTHON_EXECUTABLE and PYTHON_LIBRARY
#    are same as those used by the actual python env.

find_package(pybind11 REQUIRED CONFIG)
include_directories(${pybind11_INCLUDE_DIR})

find_package (OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message(STATUS "OpenCV_INCLUDE_DIRS => ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_LIBS => ${OpenCV_LIBS}")


pybind11_add_module(cv_wrapper pybind11_interface.cc mat_wrapper.cc)
target_link_libraries(cv_wrapper PRIVATE ${OpenCV_LIBS})
