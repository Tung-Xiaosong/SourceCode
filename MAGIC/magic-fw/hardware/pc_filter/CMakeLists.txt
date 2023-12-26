cmake_minimum_required(VERSION 3.0.2)
project(pc_filter)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        tf2
        tf2_ros
        tf2_geometry_msgs
        std_msgs
        geometry_msgs
        sensor_msgs
        nav_msgs
        diagnostic_msgs
        pcl_conversions
        pcl_ros
        message_generation
        )

generate_messages(
        DEPENDENCIES
        std_msgs
)

find_package(PCL REQUIRED QUIET)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS
        roscpp
        tf2
        tf2_ros
        tf2_geometry_msgs
        geometry_msgs
        sensor_msgs
        nav_msgs
        diagnostic_msgs
        pcl_conversions
        pcl_ros
        message_generation
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(pc_filter_node
 src/pc_node.cpp
)
add_dependencies(pc_filter_node ${${PROJECT_NAME}_EXPORTED_TARGETS}
${catkin_EXPORTED_TARGETS})
target_link_libraries(pc_filter_node
 ${catkin_LIBRARIES}
)
