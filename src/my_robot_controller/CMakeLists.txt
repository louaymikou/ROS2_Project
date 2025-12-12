cmake_minimum_required(VERSION 3.8)
project(my_robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY description launch config worlds models
  DESTINATION share/${PROJECT_NAME}
)

# Installer les scripts Python
install(
  PROGRAMS
    ps4_controller.py
    simple_arm_control.py
    keyboard_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()