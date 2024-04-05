cmake_minimum_required(VERSION 2.8.3)
project(robot_control)


find_package(catkin REQUIRED
    joint_state_controller
    velocity_controllers
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

install(
  DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
