cmake_minimum_required(VERSION 3.5)
project(niagara_nav)


find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch rviz config maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
