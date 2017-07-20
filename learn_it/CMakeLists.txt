cmake_minimum_required(VERSION 3.1)
project(learn_it)

find_package(OpenCV REQUIRED)

add_executable(learn_opencv learn_opencv.cc)
target_link_libraries(learn_opencv
  ${OpenCV_LIBS}
  ORB_SLAM2
  )
