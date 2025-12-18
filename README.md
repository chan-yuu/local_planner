# local_planner

安装spatio_temporal_voxel_layer等内容
```
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
```

```
sudo apt install ros-galactic-velodyne-simulator
```

```
ros2 launch niagara_model display.launch.py
```

```
ros2 launch niagara_nav navigation.launch.py
```
