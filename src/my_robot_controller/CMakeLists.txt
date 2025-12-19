cmake_minimum_required(VERSION 3.8)
project(my_robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Generate action interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PickAndPlace.action"
  DEPENDENCIES action_msgs std_msgs
)

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
    pick_and_place_server.py
    pick_and_place_client.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()