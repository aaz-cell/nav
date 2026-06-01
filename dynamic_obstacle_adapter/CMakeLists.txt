cmake_minimum_required(VERSION 3.0.2)
project(dynamic_obstacle_adapter)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  sensor_msgs
  tf2
)

catkin_package(
  CATKIN_DEPENDS nav_msgs roscpp sensor_msgs tf2
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(dynamic_obstacle_adapter_node src/dynamic_obstacle_adapter_node.cpp)
target_link_libraries(dynamic_obstacle_adapter_node ${catkin_LIBRARIES})
