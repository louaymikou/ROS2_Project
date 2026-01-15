cmake_minimum_required(VERSION 3.8)
project(my_robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY description launch config worlds models nodes maps
  DESTINATION share/${PROJECT_NAME}
)

# Install Python nodes from organized structure
install(
  PROGRAMS
    nodes/controllers/keyboard_controller.py
    nodes/controllers/arm_moveit_control.py
    nodes/controllers/interactive_arm_control.py
    nodes/navigation/auto_navigator.py
    nodes/workflows/pick_and_place_workflow.py
    nodes/workflows/configurable_pick_place.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()