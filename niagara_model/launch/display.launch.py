import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
# 新增 IncludeLaunchDescription 用于包含其他 launch 文件
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, IncludeLaunchDescription
# 新增 PythonLaunchDescriptionSource 用于解析 python 格式的 launch 文件
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='niagara_model').find('niagara_model')
    
    default_model_path = os.path.join(pkg_share, 'urdf/DasAutonomeAuto.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config_points.rviz')
    # world_path 不再需要，因为我们要用默认的 empty_world
    
    use_sim_time = LaunchConfiguration('use_sim_time') 

    # =======================================================================
    # 修改部分：使用标准方式启动 Gazebo (默认就是 Empty World)
    # =======================================================================
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
        ),
        # 如果将来想加载特定世界，可以在这里取消注释并修改路径：
        launch_arguments={'world': os.path.join(pkg_share, 'world/nav.world')}.items()
    )
    # =======================================================================

    map_odom_tf_node = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_odom_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )

    # Position and orientation
    # [X, Y, Z]
    position = [-10.0, 0.0, 0.5]
    # [Roll, Pitch, Yaw]
    orientation = [0.0, 0.0, 0.0]
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
        )

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_model',
        # output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'dasautonomeauto', '-x', str(position[0]), '-y', str(position[1]), '-z', str(position[2]), '-R', str(orientation[0]), '-P', str(orientation[1]), '-Y', str(orientation[2]),'-topic', '/robot_description'],
        output='screen'
    )
    
    # 控制器相关的节点保持原样，如果不需要可以注释掉
    velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'velocity_controller'],
        output='screen'
    )
    
    position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'position_controller'],
        output='screen'
    )

    localization_node = launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("robot_localization"), 'params', 'my_localization.yaml')],
    )

    return launch.LaunchDescription([
        
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        declare_use_sim_time_cmd,
        
        # 在这里启动 Gazebo
        gazebo,
        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_entity,
        TimerAction(
            actions=[
                rviz_node,
                map_odom_tf_node,
                # position_controller,
                localization_node,
            ],
            period='6.0',  
        ),
    ])