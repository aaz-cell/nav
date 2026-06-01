cmake_minimum_required(VERSION 3.0.2)
project(navigation_adapter)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  tf2
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs tf2
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(navigation_adapter_node src/navigation_adapter_node.cpp)
target_link_libraries(navigation_adapter_node ${catkin_LIBRARIES})
