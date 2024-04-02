cmake_minimum_required(VERSION 3.0.2)
project(ros_backend_interface)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  move_base_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2_msgs
  pcl_conversions
  pcl_ros
  laser_geometry
  visualization_msgs
  yhs_msgs
)

find_package(PCL REQUIRED)
#find_package(Boost REQUIRED COMPONENTS system)
#find_package(catkin REQUIRED COMPONENTS
#             message_filters)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ros_backend_interface
  CATKIN_DEPENDS geometry_msgs move_base_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf tf2_msgs
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}/yhs_msgs
)

add_executable(${PROJECT_NAME}_node src/main.cpp src/ros_backend_interface.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)

set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME ros_backend_interface)

