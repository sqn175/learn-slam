cmake_minimum_required(VERSION 2.8)

project(frontend)

include_directories(../common)

add_library(${PROJECT_NAME} STATIC
pinhole_camera.cc
cv.cc
frame.cc
frontend.cc
landmark.cc
keyframe.cc
map.cc
guided_matcher.cc
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ORB_SLAM2
  common
)
