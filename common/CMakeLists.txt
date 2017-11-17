cmake_minimum_required(VERSION 3.1)
project(common)

include_directories(
  ../frontend
)

add_library(${PROJECT_NAME} STATIC
parameters_reader.cc
helper.cc
range_searcher.cc
visualizer.cc
timer.cc
)

target_link_libraries(${PROJECT_NAME}
${Pangolin_LIBRARIES}
${OpenCV_LIBS}
${GLOG_LIBRARY}
frontend
)

# Testing
if(${BUILD_TESTS})
  #enable_testing()
  set(PROJECT_TEST_NAME ${PROJECT_NAME}_test)
  add_executable(${PROJECT_TEST_NAME}
    test/test_main.cc  
  )
  target_link_libraries(${PROJECT_TEST_NAME}
    gtest
    gmock_main
    ${PROJECT_NAME}
  )
  add_test(NAME test COMMAND ${PROJECT_TEST_NAME})
endif()

