cmake_minimum_required(VERSION 3.1)
project(slam)

add_library(${PROJECT_NAME} STATIC
slam.cc
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${GLOG_LIBRARY}
ORB_SLAM2
frontend
common
)
