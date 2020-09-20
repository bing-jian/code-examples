# Important notes:
# 1) Install pybind11 using conda
#    conda install pybind11
# 2) Config Python version accordingly when using CMake
#    make sure the PYTHON_EXECUTABLE and PYTHON_LIBRARY
#    are same as those used by the actual python env.

find_package(pybind11 REQUIRED CONFIG)
include_directories(${pybind11_INCLUDE_DIR})

# Find the numpy include directory
execute_process(
  COMMAND "${PYTHON_EXECUTABLE}" -c
      "from __future__ import print_function\ntry: import numpy; print(numpy.get_include(), end='')\nexcept:pass\n"
  OUTPUT_VARIABLE __numpy_path)

message(STATUS "__numpy_path : " ${__numpy_path})
# and the version
execute_process(
  COMMAND "${PYTHON_EXECUTABLE}" -c
      "from __future__ import print_function\ntry: import numpy; print(numpy.__version__, end='')\nexcept:pass\n"
  OUTPUT_VARIABLE __numpy_version)
message(STATUS "__numpy_version : " ${__numpy_version})

# Fouble check the required numpy header file
find_path(PYTHON_NUMPY_INCLUDE_DIR numpy/arrayobject.h
HINTS "${__numpy_path}" "${PYTHON_INCLUDE_PATH}" NO_DEFAULT_PATH)
message(STATUS "PYTHON_NUMPY_INCLUDE_DIR : " ${PYTHON_NUMPY_INCLUDE_DIR})
include_directories(${PYTHON_NUMPY_INCLUDE_DIR})


pybind11_add_module(cv_wrapper_v1 MODULE cv_wrapper_v1_interface.cc mat_wrapper.cc)
target_link_libraries(cv_wrapper_v1 PRIVATE ${OpenCV_LIBS})
set_target_properties(cv_wrapper_v1 PROPERTIES PREFIX "" INSTALL_RPATH_USE_LINK_PATH TRUE)
install(TARGETS cv_wrapper_v1 DESTINATION ${CMAKE_SOURCE_DIR}/wrapper)

pybind11_add_module(cv_wrapper_v2 MODULE cv_wrapper_v2_interface.cc ndarray_converter.cc)
target_link_libraries(cv_wrapper_v2 PRIVATE depth_utils ${OpenCV_LIBS})
set_target_properties(cv_wrapper_v2 PROPERTIES PREFIX ""  INSTALL_RPATH_USE_LINK_PATH TRUE)
install(TARGETS cv_wrapper_v2 DESTINATION ${CMAKE_SOURCE_DIR}/wrapper)
