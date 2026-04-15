cmake_minimum_required(VERSION 3.8)
project(simulation_rviz)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/rviz.py
  scripts/control.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY meshes urdf config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
