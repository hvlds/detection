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

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(filtered_camera src/filtered_camera.cpp)
target_link_libraries(filtered_camera ${catkin_LIBRARIES})

add_executable(camera_synchronizer src/camera_synchronizer.cpp)
target_link_libraries(camera_synchronizer ${catkin_LIBRARIES})

add_executable(brightness_analyser src/brightness_analyser.cpp)
target_link_libraries(brightness_analyser ${catkin_LIBRARIES})