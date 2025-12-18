cmake_minimum_required(VERSION 2.8.3)
project(frenet_planner)

set (CMAKE_CXX_STANDARD 14)
set (CMAKE_CXX_FLAGS "  -Wall -Wextra -Wpedantic  -ltbb -O3 -fopenmp -fstack-protector-all")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  nav_msgs
  std_msgs
  cmake_modules
  #costmap_converter 
  base_local_planner
  message_generation
  nav_core
  costmap_2d
  pluginlib
  visualization_msgs
  tf
  #tf_convesion
)

# Removed Python path for matplotlib (no longer needed)

# 设置TBB配置文件路径到cmake文件夹
SET (TBB_DIR  ${PROJECT_SOURCE_DIR}/cmake/TBBConfig.cmake) 
find_package(TBB)
# 添加cmake模块路径，包含自定义的cmake文件夹
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules ${PROJECT_SOURCE_DIR}/cmake)
message(STATUS "${CMAKE_MODULE_PATH}")
find_package(Boost REQUIRED COMPONENTS system thread graph)

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})	
endif()

catkin_package(
  CATKIN_DEPENDS 
	geometry_msgs nav_msgs roscpp rospy std_msgs costmap_2d base_local_planner
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(
	frenet_include src/cubic_spline_planner.cpp src/polynomials.cpp src/frenet_optimal_trajectory.cpp
	)

add_executable(frenet_planner src/frenetROS_obst.cpp)
add_executable(global_path_planner src/global_path_planner.cpp)

target_link_libraries(frenet_planner
	${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${TBB_LIBRARIES}
	frenet_include
)

target_link_libraries(global_path_planner
	${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${TBB_LIBRARIES}
	frenet_include
)

# Removed Python library configurations (no longer needed for matplotlib)

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()
