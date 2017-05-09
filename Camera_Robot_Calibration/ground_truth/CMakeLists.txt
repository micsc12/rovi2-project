cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -g -ftest-coverage -fprofile-arcs")

add_definitions(-std=c++11)

project( ground_truth )
find_package( OpenCV REQUIRED )
add_executable( ground_truth main.cpp )
#add_executable( epipolar main2.cpp)
target_link_libraries( ground_truth ${OpenCV_LIBS} )
#target_link_libraries( epipolar ${OPENCV_LIBS} )
