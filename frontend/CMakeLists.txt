cmake_minimum_required(VERSION 2.8)

project(frontend)

include_directories(
  ../common
  ../3rdparty/DBoW2/DBoW2
  )

add_library(${PROJECT_NAME} STATIC
pinhole_camera.cc
cv.cc
frame.cc
frontend.cc
mappoint.cc
keyframe.cc
map.cc
guided_matcher.cc
)
message(STATUS ${PROJECT_SOURCE_DIR})
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ORB_SLAM2
  common
  ${PROJECT_SOURCE_DIR}/../3rdparty/DBoW2/lib/libDBoW2.a
)
