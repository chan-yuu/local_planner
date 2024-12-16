cmake_minimum_required(VERSION 3.0.2)
project(obstacle_perception)
add_compile_options(-std=c++14)

## 查找catkin包，添加项目依赖的相关组件
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  pcl_conversions
  pcl_ros
  nav_msgs
  tf2_ros
  tf2_eigen
)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros tf2_ros  nav_msgs
)

## 包含头文件路径
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## 声明可执行文件
add_executable(obstacle_perception_node src/obstacle_perception_node.cpp)

## 添加链接库，确保链接所有依赖的库
target_link_libraries(obstacle_perception_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)