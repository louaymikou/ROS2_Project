cmake_minimum_required(VERSION 3.8)
project(my_robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Installer les dossiers pour qu'ils soient visibles après la compilation
install(
  DIRECTORY description launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()