cmake_minimum_required(VERSION 3.1)
project(learn_it)

add_executable(learn_opencv learn_orb.cc)
target_link_libraries(learn_opencv
  ${OpenCV_LIBS}
  ORB_SLAM2
  frontend
  )
