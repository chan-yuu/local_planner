import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

MAP_NAME='map' #playground or office_earthquake
# Global variable to specify which map to be launched in rviz
# Global variable to specify in a easier way which sensor to use for stvl layer
SENSOR = 'ackermann2' #3d or rgbd or ackermann

def generate_launch_description():
    depth_sensor = os.getenv('LINOROBOT2_DEPTH_SENSOR', '')

    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('niagara_nav'), 'rviz', 'niagara_nav.rviz']
    )

    default_map_path = PathJoinSubstitution(
        [FindPackageShare('niagara_nav'), 'maps', f'{MAP_NAME}.yaml']
    )

    # Passing parameter file.yaml according to which sensor has been selected for the stvl layer
    nav2_config_path = PathJoinSubstitution(
        [FindPackageShare('niagara_nav'), 'config', f'navigation_{SENSOR}.yaml']
    )

    # Map to odom tf publisher node - commented out as AMCL handles this transform
    # map_odom_tf_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='map_odom_tf_publisher',
    #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    #     parameters=[{'use_sim_time': LaunchConfiguration("sim")}],
    #     output='screen'
    # )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='sim', 
            default_value='false',
            description='Enable use_sime_time to true'
        ),

        DeclareLaunchArgument(
            name='rviz', 
            default_value='true',
            description='Run rviz'
        ),

       DeclareLaunchArgument(
            name='map', 
            default_value=default_map_path,
            description='Navigation map path'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            launch_arguments={
                'map': LaunchConfiguration("map"),
                'use_sim_time': LaunchConfiguration("sim"),
                'params_file': nav2_config_path,
                'use_localization': 'false'
            }.items()
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_test',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[{'use_sim_time': LaunchConfiguration("sim")}]
        ),

        # map_odom_tf_node  # Commented out - AMCL handles map->odom transform
    ])
