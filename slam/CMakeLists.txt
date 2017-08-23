cmake_minimum_required(VERSION 3.1)
project(slam)

add_library(${PROJECT_NAME} STATIC
slam.cc
)
