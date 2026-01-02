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
  DIRECTORY description launch config worlds models action nodes
  DESTINATION share/${PROJECT_NAME}
)

# Install Python nodes from organized structure
install(
  PROGRAMS
    nodes/controllers/ps4_controller.py
    nodes/controllers/keyboard_controller.py
    nodes/mappers/auto_mapper.py
    nodes/mappers/auto_mapper_detailed.py
    nodes/mappers/wall_follower_mapper.py
    nodes/navigation/navigate_to_package.py
    nodes/navigation/set_initial_pose.py
    nodes/navigation/backward_to_wall.py
    nodes/arm_control/simple_arm_control.py
    nodes/arm_control/pick_package.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install scripts from scripts/ folder
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