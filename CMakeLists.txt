cmake_minimum_required(VERSION 3.0.2)
project(calibration)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  image_transport
  cv_bridge
  tf
  livox_ros_driver
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs livox_ros_driver
  DEPENDS EIGEN3 PCL 
  INCLUDE_DIRS include
)

add_library(LidarCameraFusion SHARED src/io_utils.cpp src/lidar_camera_fusion.cpp)
target_link_libraries(LidarCameraFusion ${OpenCV_LIBS} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

