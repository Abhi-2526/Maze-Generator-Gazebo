cmake_minimum_required(VERSION 3.0.2)
project(Maze-Generator-Gazebo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
  sensor_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/maze_solver.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
