cmake_minimum_required(VERSION 3.1)
project(ORB_SLAM2)

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

include_directories(
#${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
#${EIGEN3_INCLUDE_DIR}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} STATIC
ORBextractor.cc
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
)
