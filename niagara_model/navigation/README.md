# Navigation2 阿克曼车辆集成使用说明

本文档介绍如何使用为阿克曼转向车辆定制的Navigation2配置。

## 目录结构

```
navigation/
├── config/
│   ├── nav2_params.yaml          # Navigation2主配置文件
│   └── stvl_layer_config.yaml    # STVL Layer专用配置
├── launch/
│   └── nav2_bringup.launch.py    # Navigation2启动文件
├── behavior_trees/
│   └── ackermann_nav_bt.xml      # 阿克曼转向行为树
├── nav2_ackermann_checker.py     # 系统检查脚本
└── README.md                     # 本文档
```

## 系统要求

### 必需组件
- ROS2 Galactic或更高版本
- Navigation2包
- STVL Layer插件
- 3D雷达点云数据（/rslidar_points话题）
- 里程计数据（/odom话题）
- 地图数据（/map话题）

### TF变换链要求
```
map -> odom -> base_footprint -> base_link -> rslidar
```

## 快速开始

### 1. 系统检查

在启动Navigation2之前，建议先运行系统检查脚本：

```bash
# 进入工作空间
cd ~/Documents/cat_ws

# 编译工作空间（如果需要）
colcon build

# 设置环境
source install/setup.bash

# 运行检查脚本
python3 src/car_demo_ros2_acker/niagara_model/navigation/nav2_ackermann_checker.py
```

检查脚本将验证：
- 必需话题是否存在并有数据
- TF变换链是否完整
- Navigation2节点状态
- 阿克曼转向特定配置
- STVL Layer要求

### 2. 启动Navigation2

#### 方法一：使用我们的自定义启动文件

```bash
# 启动Navigation2（假设地图服务器已运行）
ros2 launch niagara_model nav2_bringup.launch.py

# 如果需要指定参数文件路径
ros2 launch niagara_model nav2_bringup.launch.py \
    params_file:=/path/to/your/nav2_params.yaml

# 如果需要指定行为树文件
ros2 launch niagara_model nav2_bringup.launch.py \
    default_nav_to_pose_bt_xml:=/path/to/your/ackermann_nav_bt.xml
```

#### 方法二：使用标准Navigation2启动文件

```bash
# 使用我们的配置文件启动标准Navigation2
ros2 launch nav2_bringup navigation_launch.py \
    params_file:=src/car_demo_ros2_acker/niagara_model/navigation/config/nav2_params.yaml
```

### 3. 启动地图服务器（如果需要）

```bash
# 启动地图服务器
ros2 launch nav2_bringup bringup_launch.py \
    map:=/path/to/your/map.yaml
```

### 4. 启动RViz进行可视化

```bash
# 启动RViz with Navigation2配置
ros2 launch nav2_bringup rviz_launch.py
```

## 配置说明

### 阿克曼转向特定配置

我们的配置针对阿克曼转向车辆进行了以下优化：

1. **控制器配置**
   - 使用`RegulatedPurePursuitController`
   - 禁用倒车功能（`allow_reverse: false`）
   - 设置最大转向角度限制
   - 优化路径跟踪参数

2. **规划器配置**
   - 使用`NavfnPlanner`进行全局路径规划
   - 配置适合阿克曼车辆的路径平滑参数

3. **行为树配置**
   - 定制的恢复行为（避免原地旋转）
   - 前进优先策略
   - 适合的避障行为

### STVL Layer配置

针对3D雷达点云的配置：

1. **点云处理**
   - 体素大小：0.05m
   - 高度过滤：-0.5m到2.5m
   - 距离过滤：0.3m到15.0m

2. **障碍物检测**
   - 前方重点检测区域：8m x 3m x 2.5m
   - 转向时检测区域扩展
   - 速度自适应检测范围

## 参数调优

### 控制器参数调优

根据您的车辆特性，可能需要调整以下参数：

```yaml
controller_server:
  ros__parameters:
    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      # 调整这些参数以匹配您的车辆
      max_linear_accel: 2.5          # 最大线性加速度
      max_linear_decel: 2.5          # 最大线性减速度
      max_angular_accel: 3.2         # 最大角加速度
      max_robot_pose_search_dist: 10.0  # 机器人位姿搜索距离
      lookahead_dist: 0.6            # 前瞻距离
      min_lookahead_dist: 0.3        # 最小前瞻距离
      max_lookahead_dist: 0.9        # 最大前瞻距离
```

### STVL Layer参数调优

根据您的雷达特性调整：

```yaml
stvl_layer:
  rslidar_points:
    max_obstacle_height: 3.0       # 根据环境调整
    min_obstacle_height: 0.1       # 根据地面情况调整
    obstacle_range: 15.0           # 根据雷达性能调整
    voxel_size: 0.05              # 根据精度需求调整
```

## 故障排除

### 常见问题

1. **Navigation2节点无法启动**
   - 检查参数文件路径是否正确
   - 确保所有依赖包已安装
   - 检查ROS2环境是否正确设置

2. **路径规划失败**
   - 检查地图是否正确加载
   - 验证起点和终点是否在自由空间
   - 检查costmap配置

3. **控制器无法跟踪路径**
   - 检查cmd_vel话题是否正确连接
   - 调整控制器参数
   - 验证里程计数据质量

4. **STVL Layer无数据**
   - 检查点云话题是否发布
   - 验证TF变换是否正确
   - 检查点云数据格式

### 调试命令

```bash
# 检查话题
ros2 topic list
ros2 topic echo /rslidar_points --once
ros2 topic echo /odom --once

# 检查TF
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo map base_footprint

# 检查节点
ros2 node list
ros2 node info /bt_navigator

# 检查参数
ros2 param list /controller_server
ros2 param get /controller_server FollowPath.plugin
```

## 性能优化建议

1. **计算资源优化**
   - 根据硬件性能调整更新频率
   - 适当调整体素大小和检测范围
   - 使用多线程处理（如果支持）

2. **网络优化**
   - 使用适当的QoS设置
   - 考虑点云数据压缩
   - 优化话题发布频率

3. **算法优化**
   - 根据场景选择合适的规划器
   - 调整行为树逻辑
   - 优化恢复行为

## 扩展功能

### 添加新的控制器

如果需要使用其他控制器，可以在`nav2_params.yaml`中添加：

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath", "YourNewController"]
    YourNewController:
      plugin: "your_controller_package::YourControllerClass"
      # 控制器特定参数
```

### 添加新的规划器

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased", "YourNewPlanner"]
    YourNewPlanner:
      plugin: "your_planner_package::YourPlannerClass"
      # 规划器特定参数
```

## 支持和贡献

如果您遇到问题或有改进建议，请：

1. 首先运行检查脚本诊断问题
2. 查看日志文件获取详细错误信息
3. 参考Navigation2官方文档
4. 在项目仓库中提交issue

## 版本历史

- v1.0 (2024): 初始版本，支持基本的阿克曼转向导航功能

---

**注意**: 本配置专为阿克曼转向车辆设计，如果您使用差速驱动机器人，请使用标准的Navigation2配置。