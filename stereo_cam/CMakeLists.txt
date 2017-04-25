cmake_minimum_required(VERSION 2.8.3)
project(stereo_cam)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

set(CMAKE_CXX_FLAGS "-std=c++11")


find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    #OpenCV REQUIRED
    image_transport
    cv_bridge
    message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   point.msg
#   Message1.msg
#   Message2.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  stereo_cam
)

catkin_package(
  LIBRARIES stereo_cam
  CATKIN_DEPENDS cv_bridge image_transport roscpp std_msgs
  DEPENDS system_lib
)


find_package( OpenCV REQUIRED )
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_LIBS}
    include
)

## Declare a C++ library
add_library(stereo_cam
  src/stereo.cpp
#  src/red_ball_node_right.cpp
)

add_dependencies(stereo_cam stereo_cam_generate_dependencies_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



add_executable(${PROJECT_NAME}_node src/stereo.cpp)


add_dependencies(${PROJECT_NAME}_node stereo_cam_generate_dependencies_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)

target_link_libraries(stereo_cam
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)
