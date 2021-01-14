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

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})