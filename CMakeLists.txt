cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
project(Learn_SLAM)

#SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11 -pthread")
set (CMAKE_CXX_STANDARD 11)

set(CMAKE_BUILD_TYPE Debug)

option (BUILD_TESTS "Builds all gtests" ON) 
option (USE_TIMER "Timing execution cost" ON)

if (${USE_TIMER})
  add_definitions(-DUSE_TIMER)
endif()

if(${BUILD_TESTS})
  include(CTest)
  if (CMAKE_VERSION VERSION_LESS 3.2)
    set(UPDATE_DISCONNECTED_IF_AVAILABLE "")
  else()
    set(UPDATE_DISCONNECTED_IF_AVAILABLE "UPDATE_DISCONNECTED 1")
  endif()

  include(${PROJECT_SOURCE_DIR}/cmake_modules/DownloadProject.cmake)
  # Download Google Test
  download_project(PROJ                googletest
                  GIT_REPOSITORY      https://github.com/google/googletest.git
                  GIT_TAG             master
                  ${UPDATE_DISCONNECTED_IF_AVAILABLE}
  )

  # Prevent GoogleTest from overriding our compiler/linker options
  # when building with Visual Studio
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

  add_subdirectory(${googletest_SOURCE_DIR} ${googletest_BINARY_DIR})

  # When using CMake 2.8.11 or later, header path dependencies
  # are automatically added to the gtest and gmock targets.
  # For earlier CMake versions, we have to explicitly add the
  # required directories to the header search path ourselves.
  if (CMAKE_VERSION VERSION_LESS 2.8.11)
      include_directories("${gtest_SOURCE_DIR}/include"
                          "${gmock_SOURCE_DIR}/include")
  endif()
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

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
include_directories(
${Pangolin_INCLUDE_DIRS})
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

