cmake_minimum_required(VERSION 3.8)
project(robotnik_simulation_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  maps
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)

endif()

ament_package()
