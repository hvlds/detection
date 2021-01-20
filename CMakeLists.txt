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
  message_generation
)

find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(filtered_camera_node src/filtered_camera_node.cpp src/filtered_camera.cpp)
target_link_libraries(filtered_camera_node ${catkin_LIBRARIES})

add_executable(camera_synchronizer_node src/camera_synchronizer_node.cpp)
target_link_libraries(camera_synchronizer_node ${catkin_LIBRARIES})

add_executable(brightness_analyser_node src/brightness_analyser_node.cpp)
target_link_libraries(brightness_analyser_node ${catkin_LIBRARIES})