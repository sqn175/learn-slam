cmake_minimum_required(VERSION 2.8)

project(frontend)

add_library(${PROJECT_NAME} STATIC
pinhole_camera.cc
cv.cc
camera_measurement.cc
frontend.cc
landmark.cc
keyframe.cc
map.cc
)

