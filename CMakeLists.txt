cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
project(Learn_SLAM)

set (CMAKE_CXX_STANDARD 11)

set(CMAKE_BUILD_TYPE Debug)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules")

# Require OpenCV
find_package(OpenCV 3.3.0 REQUIRED)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()
message(STATUS "opencv:" ${OpenCV_VERSION})
message(STATUS "opencv include path:"  ${OpenCV_INCLUDE_DIRS})

# Require Eigen3
find_package(Eigen3 3.1.0 REQUIRED)

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})
message(STATUS "pangolin include path:"  ${Pangolin_INCLUDE_DIRS})

# Require Glog
find_package(Glog REQUIRED)
include_directories(BEFORE ${GLOG_INCLUDE_DIRS})

include_directories(3rdparty/ORB_SLAM2_modified/)
add_subdirectory(3rdparty/ORB_SLAM2_modified)

include_directories(common/)
add_subdirectory(common)

include_directories(frontend/)
add_subdirectory(frontend)

include_directories(slam/)
add_subdirectory(slam)

# build learn_it examples
set(BUILD_LEARN_EXAMPLE 1)
if (BUILD_LEARN_EXAMPLE)
  add_subdirectory(learn_it)
  message(STATUS "build example")
endif()

