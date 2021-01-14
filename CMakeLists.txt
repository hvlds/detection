cmake_minimum_required(VERSION 3.0.2)
project(detection)

find_package(catkin REQUIRED COMPONENTS
  apriltag_ros
  openni2_camera
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(filter src/filter.cpp)
target_link_libraries(filter ${catkin_LIBRARIES})