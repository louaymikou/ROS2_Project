cmake_minimum_required(VERSION 3.8)
project(my_robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY description launch config worlds models nodes
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
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()