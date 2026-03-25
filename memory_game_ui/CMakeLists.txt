cmake_minimum_required(VERSION 3.0.2)
project(memory_game_ui)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

# Install Python scripts so rosrun can find them
catkin_install_python(PROGRAMS
  scripts/game_ui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
