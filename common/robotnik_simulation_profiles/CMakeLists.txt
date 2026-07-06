cmake_minimum_required(VERSION 3.8)
project(robotnik_simulation_profiles)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  rb1
  rbcar
  rbfiqus
  rbkairos
  rbrobout
  rbsummit
  rbsummit_steel
  rbtheron
  rbvogui
  rbvogui_xl
  rbwatcher
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
