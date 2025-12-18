#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Navigation2 阿克曼车辆启动文件
启动完整的Navigation2导航栈，适配阿克曼转向车辆

作者: Navigation2集成团队
版本: 1.0
日期: 2024
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包路径
    niagara_model_dir = get_package_share_directory('niagara_model')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # 声明启动参数
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(niagara_model_dir, 'navigation', 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_bt_xml_cmd = DeclareLaunchArgument(
        'default_nav_to_pose_bt_xml',
        default_value=os.path.join(niagara_model_dir, 'navigation', 'behavior_trees', 'ackermann_nav_bt.xml'),
        description='Full path to the behavior tree xml file to use')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', 
        default_value='False',
        description='Whether to use composed bringup')

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name',
        default_value='nav2_container',
        description='the name of conatiner that nodes will load in if use composition')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', 
        default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', 
        default_value='info',
        description='log level')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', 
        default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_lifecycle_mgr_cmd = DeclareLaunchArgument(
        'use_lifecycle_mgr', 
        default_value='true',
        description='Whether to launch the lifecycle manager')

    # 获取启动配置
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    default_nav_to_pose_bt_xml = LaunchConfiguration('default_nav_to_pose_bt_xml')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    autostart = LaunchConfiguration('autostart')
    use_lifecycle_mgr = LaunchConfiguration('use_lifecycle_mgr')

    # 重写YAML文件以包含启动时参数
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'default_nav_to_pose_bt_xml': default_nav_to_pose_bt_xml,
        'autostart': autostart,
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    # 设置环境变量
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # Map到Odom的TF发布节点（identity transform）
    map_odom_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_odom_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Navigation2节点组
    bringup_cmd_group = GroupAction([
        # 控制器服务器
        Node(
            condition=IfCondition(PythonExpression(['not ', use_composition])),
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=[('/tf', 'tf'),
                       ('/tf_static', 'tf_static')]),

        # 平滑器服务器 - 已禁用（包不存在）
        # Node(
        #     condition=IfCondition(PythonExpression(['not ', use_composition])),
        #     package='nav2_smoother',
        #     executable='smoother_server',
        #     name='smoother_server',
        #     output='screen',
        #     respawn=use_respawn,
        #     respawn_delay=2.0,
        #     parameters=[configured_params],
        #     arguments=['--ros-args', '--log-level', log_level],
        #     remappings=[('/tf', 'tf'),
        #                ('/tf_static', 'tf_static')]),

        # 规划器服务器
        Node(
            condition=IfCondition(PythonExpression(['not ', use_composition])),
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=[('/tf', 'tf'),
                       ('/tf_static', 'tf_static')]),

        # 行为服务器 - 已禁用（包不存在）
        # Node(
        #     condition=IfCondition(PythonExpression(['not ', use_composition])),
        #     package='nav2_behaviors',
        #     executable='behavior_server',
        #     name='behavior_server',
        #     output='screen',
        #     respawn=use_respawn,
        #     respawn_delay=2.0,
        #     parameters=[configured_params],
        #     arguments=['--ros-args', '--log-level', log_level],
        #     remappings=[('/tf', 'tf'),
        #                ('/tf_static', 'tf_static')]),

        # BT导航器
        Node(
            condition=IfCondition(PythonExpression(['not ', use_composition])),
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=[('/tf', 'tf'),
                       ('/tf_static', 'tf_static')]),

        # 路径点跟随器
        Node(
            condition=IfCondition(PythonExpression(['not ', use_composition])),
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=[('/tf', 'tf'),
                       ('/tf_static', 'tf_static')]),

        # 速度平滑器 - 已禁用（包不存在）
        # Node(
        #     condition=IfCondition(PythonExpression(['not ', use_composition])),
        #     package='nav2_velocity_smoother',
        #     executable='velocity_smoother',
        #     name='velocity_smoother',
        #     output='screen',
        #     respawn=use_respawn,
        #     respawn_delay=2.0,
        #     parameters=[configured_params],
        #     arguments=['--ros-args', '--log-level', log_level],
        #     remappings=[('/tf', 'tf'),
        #                ('/tf_static', 'tf_static'),
        #                ('cmd_vel', 'cmd_vel_nav'),
        #                ('cmd_vel_smoothed', 'cmd_vel')]),

        # 碰撞监控器 - 已禁用（包不存在）
        # Node(
        #     condition=IfCondition(PythonExpression(['not ', use_composition])),
        #     package='nav2_collision_monitor',
        #     executable='collision_monitor',
        #     name='collision_monitor',
        #     output='screen',
        #     respawn=use_respawn,
        #     respawn_delay=2.0,
        #     parameters=[configured_params],
        #     arguments=['--ros-args', '--log-level', log_level]),

        # 生命周期管理器
        Node(
            condition=IfCondition(use_lifecycle_mgr),
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{'use_sim_time': use_sim_time},
                       {'autostart': autostart},
                       {'node_names': [
                           'controller_server',
                           'planner_server',
                           'bt_navigator',
                           'waypoint_follower'
                       ]}]),
    ])

    # 组合节点配置（可选）
    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_name,
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[configured_params],
                remappings=[('/tf', 'tf'),
                           ('/tf_static', 'tf_static')]),
            # ComposableNode(
            #     package='nav2_smoother',
            #     plugin='nav2_smoother::SmootherServer',
            #     name='smoother_server',
            #     parameters=[configured_params],
            #     remappings=[('/tf', 'tf'),
            #                ('/tf_static', 'tf_static')]),
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[configured_params],
                remappings=[('/tf', 'tf'),
                           ('/tf_static', 'tf_static')]),
            # ComposableNode(
            #     package='nav2_behaviors',
            #     plugin='behavior_server::BehaviorServer',
            #     name='behavior_server',
            #     parameters=[configured_params],
            #     remappings=[('/tf', 'tf'),
            #                ('/tf_static', 'tf_static')]),
            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[configured_params],
                remappings=[('/tf', 'tf'),
                           ('/tf_static', 'tf_static')]),
            ComposableNode(
                package='nav2_waypoint_follower',
                plugin='nav2_waypoint_follower::WaypointFollower',
                name='waypoint_follower',
                parameters=[configured_params],
                remappings=[('/tf', 'tf'),
                           ('/tf_static', 'tf_static')]),
            # ComposableNode(
            #     package='nav2_velocity_smoother',
            #     plugin='nav2_velocity_smoother::VelocitySmoother',
            #     name='velocity_smoother',
            #     parameters=[configured_params],
            #     remappings=[('/tf', 'tf'),
            #                ('/tf_static', 'tf_static'),
            #                ('cmd_vel', 'cmd_vel_nav'),
            #                ('cmd_vel_smoothed', 'cmd_vel')]),
            # ComposableNode(
            #     package='nav2_collision_monitor',
            #     plugin='nav2_collision_monitor::CollisionMonitor',
            #     name='collision_monitor',
            #     parameters=[configured_params]),
        ],
    )

    # 创建启动描述
    ld = LaunchDescription()

    # 设置环境变量
    ld.add_action(stdout_linebuf_envvar)

    # 声明启动参数
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_lifecycle_mgr_cmd)

    # 添加节点
    ld.add_action(map_odom_tf_node)
    ld.add_action(bringup_cmd_group)
    ld.add_action(load_composable_nodes)

    return ld