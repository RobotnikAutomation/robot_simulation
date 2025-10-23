cmake_minimum_required(VERSION 3.8)
project(robotnik_gazebo_ignition)

find_package(ament_cmake REQUIRED)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files
install(
  DIRECTORY config launch models worlds scripts
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
