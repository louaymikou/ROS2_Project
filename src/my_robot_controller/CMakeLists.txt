cmake_minimum_required(VERSION 3.8)
project(my_robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generate custom actions
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PickPlace.action"
  "action/MoveArm.action"
  DEPENDENCIES geometry_msgs
)

install(
  DIRECTORY description launch config worlds models action rviz
  DESTINATION share/${PROJECT_NAME}
)

# Create scripts directory install
install(
  DIRECTORY scripts/
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
  PATTERN "*.py"
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