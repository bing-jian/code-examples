CMAKE_MINIMUM_REQUIRED(VERSION 3.10)
IF(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
ENDIF(COMMAND cmake_policy)

PROJECT(example)
SET(CMAKE_CXX_STANDARD 17)
SET(CMAKE_POSITION_INDEPENDENT_CODE ON)
if (MSVC)
    add_compile_definitions(_USE_MATH_DEFINES)
endif()

# Set a default build type if none was specified
set(default_build_type "Release")
if(NOT CMAKE_BUILD_TYPE AND NOT WIN32)
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
      STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif()

SET(GCC_FASTMATH_FLAGS "-ffast-math -funsafe-math-optimizations -msse4.2")
SET(GCC_WARNING_FLAGS "-Wuninitialized -Wno-unused-variable")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_WARNING_FLAGS} ${GCC_FASTMATH_FLAGS}")

find_package (Eigen3 3.3.90 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIR})

if (Eigen3_FOUND)
  message("-- Found Eigen version ${Eigen3_VERSION}: ${Eigen3_DIR}")
  if (CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64.*|AARCH64.*)" AND
      Eigen3_VERSION VERSION_LESS 3.3.4)
    # As per issue #289: https://github.com/ceres-solver/ceres-solver/issues/289
    # the bundle_adjustment_test will fail for Eigen < 3.3.4 on aarch64.
    message(FATAL_ERROR "-- Ceres requires Eigen version >= 3.3.4 on aarch64. "
      "Detected version of Eigen is: ${Eigen3_VERSION}.")
  endif()

  if (EIGENSPARSE)
    message("-- Enabling use of Eigen as a sparse linear algebra library.")
    list(APPEND CERES_COMPILE_OPTIONS CERES_USE_EIGEN_SPARSE)
  else (EIGENSPARSE)
    message("-- Disabling use of Eigen as a sparse linear algebra library.")
    message("   This does not affect the covariance estimation algorithm ")
    message("   which can still use the EIGEN_SPARSE_QR algorithm.")
    add_definitions(-DEIGEN_MPL2_ONLY)
  endif (EIGENSPARSE)
endif (Eigen3_FOUND)


find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# Find OpenCV -DOpenCV_DIR=...
#find_package (OpenCV REQUIRED)
#message(STATUS "OpenCV_INCLUDE_DIRS => ${OpenCV_INCLUDE_DIRS}")
#message(STATUS "OpenCV_LIBS : ${OpenCV_LIBS}")
#include_directories(${OpenCV_INCLUDE_DIRS})

find_package(glog REQUIRED)
find_package(gflags REQUIRED)
include_directories(${GFLAGS_INCLUDE_DIRS})
message(STATUS "GFLAGS_INCLUDE_DIRS => ${GFLAGS_INCLUDE_DIRS}")

include_directories(${CMAKE_CURRENT_BINARY_DIR}/core)

add_executable(pose_graph_3d pose_graph_3d.cc)
target_link_libraries(pose_graph_3d ${CERES_LIBRARIES} gflags)


add_executable(read_graph read_graph.cc)
target_link_libraries(read_graph proto gflags glog::glog)


add_subdirectory(core)


