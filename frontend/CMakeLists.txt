cmake_minimum_required(VERSION 2.8)

project(frontend)

# require Eigen3
find_package(Eigen3 REQUIRED)

# build the library
add_library(${PROJECT_NAME} STATIC
  src/pinhole_camera.cc
)
