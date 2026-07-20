cmake_minimum_required(VERSION 3.0.2)
project(sentry_nav)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  teb_local_planner
  tf
  tf2
  tf2_geometry_msgs
  tf2_ros
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(trans_tf_2d src/Trans_TF_2d.cpp)
target_link_libraries(trans_tf_2d ${catkin_LIBRARIES})

add_executable(isweep_teb_local_planner_node src/Isweep_TEB_Local_Planner.cpp)
target_link_libraries(isweep_teb_local_planner_node ${catkin_LIBRARIES})

install(TARGETS trans_tf_2d isweep_teb_local_planner_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch param rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/experiment_recorder.py
  scripts/fake_start_pose.py
  scripts/plan_timer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/launch_isweep_clean.sh
  scripts/ros_noetic_env.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
