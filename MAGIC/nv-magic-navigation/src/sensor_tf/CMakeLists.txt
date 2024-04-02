cmake_minimum_required(VERSION 3.0.2)
project(sensor_tf)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  tf2_ros
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})


add_executable(sensor_tf_node src/sensor_tf_node.cpp)


target_link_libraries(sensor_tf_node ${catkin_LIBRARIES})

