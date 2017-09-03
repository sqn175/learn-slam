cmake_minimum_required(VERSION 3.1)
project(common)

add_library(${PROJECT_NAME} STATIC
parameters_reader.cc
helper.cc
)
