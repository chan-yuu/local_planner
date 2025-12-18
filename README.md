<!--
 * @Author: CYUN && cyun@tju.enu.cn
 * @Date: 2025-12-18 22:06:00
 * @LastEditors: CYUN && cyun@tju.enu.cn
 * @LastEditTime: 2025-12-18 22:36:54
 * @FilePath: /undefined/home/cyun/Videos/local_planner/src/README.md
 * @Description: 
 * 
 * Copyright (c) 2025 by Tianjin University, All Rights Reserved. 
-->
# local_planner

安装spatio_temporal_voxel_layer等内容
sudo apt install -y \
    ros-galactic-spatio-temporal-voxel-layer
    ros-galactic-nav*
    ros-galactic-navigation2 \
    ros-galactic-nav2-bringup \
    ros-galactic-geometry2 \
    ros-galactic-pcl-ros \
    ros-galactic-tf2-geometry-msgs \
    ros-galactic-libg2o \
    libpcl-dev \
    libeigen3-dev

ros2 launch niagara_model display.launch.py

ros2 launch niagara_nav navigation.launch.py 