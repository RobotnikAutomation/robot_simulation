cmake_minimum_required(VERSION 3.5)
project(robot_description_simulation)

find_package(ament_cmake REQUIRED)
ament_package_xml()

install(
    DIRECTORY launch robots ros2_control
    DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
    DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
