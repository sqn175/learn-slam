cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
project(Learn_SLAM)

set (CMAKE_CXX_STANDARD 11)

set(CMAKE_BUILD_TYPE Debug)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules")

# build ORB_SLAM2_modified
add_subdirectory(3rdparty/ORB_SLAM2_modified)

# build learn_it examples
set(BUILD_LEARN_EXAMPLE 1)
if (BUILD_LEARN_EXAMPLE)
  add_subdirectory(learn_it)
  message(STATUS "build example")
endif()

