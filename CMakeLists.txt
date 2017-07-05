cmake_minimum_required(VERSION 2.8)
project(Learn_SLAM)

find_package(Eigen3 3.3.3)
if(NOT Eigen3_FOUND)
  message("Not found ..")
endif()