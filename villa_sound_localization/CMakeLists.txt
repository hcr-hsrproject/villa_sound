cmake_minimum_required(VERSION 2.8.3)
project(villa_sound_localization)


find_package(catkin REQUIRED COMPONENTS rospy)


catkin_package(
  DEPENDS hark_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_python_setup()

#install(PROGRAMS
#   scripts/move_robot.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )


