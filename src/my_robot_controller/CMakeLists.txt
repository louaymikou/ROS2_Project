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
  DIRECTORY description launch config worlds models action
  DESTINATION share/${PROJECT_NAME}
)

# Installer les scripts Python du dossier principal
install(
  PROGRAMS
    ps4_controller.py
    simple_arm_control.py
    keyboard_controller.py
    auto_mapper.py
    auto_mapper_detailed.py
  DESTINATION lib/${PROJECT_NAME}
)

# Installer les scripts du dossier scripts/
install(
  PROGRAMS
    scripts/arm_action_server.py
    scripts/pick_place_action_server.py
    scripts/mission_orchestrator.py
    scripts/autonomous_pick_and_store.py
    scripts/autonomous_explorer.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()