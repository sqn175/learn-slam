cmake_minimum_required(VERSION 3.1)
project(common)

include_directories(../frontend)

add_library(${PROJECT_NAME} STATIC
parameters_reader.cc
helper.cc
visualizer.cc
range_searcher.cc
)
