cmake_minimum_required(VERSION 3.0.2)
project(planner_bridge)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(planner_bridge_node src/planner_bridge_node.cpp)
target_link_libraries(planner_bridge_node ${catkin_LIBRARIES})
