cmake_minimum_required(VERSION 3.1)
project(learn_it)

# add_executable(learn_opencv learn_orb.cc)
# target_link_libraries(learn_opencv
#   ${OpenCV_LIBS}
#   ORB_SLAM2
#   frontend
#   )

# add_executable(learn_frontend learn_frontend.cc)
# target_link_libraries(learn_frontend
#   ${OpenCV_LIBS}
#   ${GLOG_LIBRARY}
#   ORB_SLAM2
#   frontend
#   common
#   )

# add_executable(learn_pangolin learn_pangolin.cc)
# target_link_libraries(learn_pangolin
#   ${OpenCV_LIBS}
#   ${GLOG_LIBRARY}
#   ORB_SLAM2
#   frontend
#   common
#   )

add_executable(slam_app slam_app.cc)
target_link_libraries(slam_app
  ${OpenCV_LIBS}
  ${GLOG_LIBRARY}
  ORB_SLAM2
  frontend
  common
  slam
  )

add_executable(testdebug test.cc)
add_executable(evaluation evaluate_euroc.cc)
target_link_libraries(evaluation
  ${OpenCV_LIBS}
  ${GLOG_LIBRARY}
)
