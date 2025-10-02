cmake_minimum_required(VERSION 3.8)
project(robotnik_o3de)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files
install(
  DIRECTORY config launch scripts
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
