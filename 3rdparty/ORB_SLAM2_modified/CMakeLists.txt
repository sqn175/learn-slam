cmake_minimum_required(VERSION 3.1)
project(ORB_SLAM2)

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})


include_directories(
${PROJECT_SOURCE_DIR}
#${PROJECT_SOURCE_DIR}/include
#${EIGEN3_INCLUDE_DIR}
../../frontend/
../../common/
../g2o/g2o/
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} STATIC
ORBextractor.cc
Optimizer.cc
Converter.cc
Initializer.cc
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
#${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/../g2o/lib/libg2o.a
${PROJECT_SOURCE_DIR}/../DBoW2/lib/libDBoW2.a
)
