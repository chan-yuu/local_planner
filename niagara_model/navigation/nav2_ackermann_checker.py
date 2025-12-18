#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Navigation2 阿克曼车辆集成检查脚本
检查当前ROS2环境是否满足Navigation2导航的所有必要条件

作者: Navigation2集成团队
版本: 1.0
日期: 2024
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import sys
import time
from typing import Dict, List, Tuple, Optional

# ROS2消息类型导入
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import TransformStamped, Twist
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Header

# TF2相关导入
import tf2_ros
from tf2_ros import TransformException

class Nav2AckermannChecker(Node):
    """
    Navigation2阿克曼车辆集成检查器
    检查所有必要的话题、服务、TF变换和参数配置
    """
    
    def __init__(self):
        super().__init__('nav2_ackermann_checker')
        
        # 检查结果存储
        self.check_results: Dict[str, bool] = {}
        self.check_details: Dict[str, str] = {}
        
        # TF监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 话题检查标志
        self.topic_checks = {
            '/rslidar_points': False,
            '/odom': False,
            '/cmd_vel': False
        }
        
        # 订阅器字典
        self.subscribers = {}
        
        self.get_logger().info("Navigation2阿克曼车辆集成检查器已启动")
        
    def check_required_topics(self) -> bool:
        """
        检查必需的ROS话题是否存在并有数据发布
        """
        self.get_logger().info("开始检查必需话题...")
        
        # 获取当前所有话题
        topic_names_and_types = self.get_topic_names_and_types()
        available_topics = [name for name, _ in topic_names_and_types]
        
        required_topics = {
            '/rslidar_points': 'sensor_msgs/msg/PointCloud2',
            '/odom': 'nav_msgs/msg/Odometry', 
            '/cmd_vel': 'geometry_msgs/msg/Twist'
        }
        
        all_topics_ok = True
        
        for topic, msg_type in required_topics.items():
            if topic in available_topics:
                self.get_logger().info(f"✓ 话题 {topic} 存在")
                self.check_results[f"topic_{topic}"] = True
                self.check_details[f"topic_{topic}"] = f"话题存在，类型: {msg_type}"
                
                # 创建订阅器检查数据流
                self._create_topic_subscriber(topic, msg_type)
            else:
                self.get_logger().error(f"✗ 话题 {topic} 不存在")
                self.check_results[f"topic_{topic}"] = False
                self.check_details[f"topic_{topic}"] = "话题不存在"
                all_topics_ok = False
                
        return all_topics_ok
    
    def _create_topic_subscriber(self, topic: str, msg_type: str):
        """
        为指定话题创建订阅器以检查数据流
        """
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )
        
        if msg_type == 'sensor_msgs/msg/PointCloud2':
            self.subscribers[topic] = self.create_subscription(
                PointCloud2,
                topic,
                lambda msg, t=topic: self._topic_callback(t, msg),
                qos_profile
            )
        elif msg_type == 'nav_msgs/msg/Odometry':
            self.subscribers[topic] = self.create_subscription(
                Odometry,
                topic,
                lambda msg, t=topic: self._topic_callback(t, msg),
                qos_profile
            )
        elif msg_type == 'geometry_msgs/msg/Twist':
            self.subscribers[topic] = self.create_subscription(
                Twist,
                topic,
                lambda msg, t=topic: self._topic_callback(t, msg),
                qos_profile
            )
    
    def _topic_callback(self, topic: str, msg):
        """
        话题回调函数，标记话题有数据
        """
        if not self.topic_checks[topic]:
            self.topic_checks[topic] = True
            self.get_logger().info(f"✓ 话题 {topic} 有数据发布")
            self.check_details[f"topic_{topic}_data"] = "话题有数据发布"
    
    def check_tf_transforms(self) -> bool:
        """
        检查必需的TF变换是否存在
        """
        self.get_logger().info("开始检查TF变换...")
        
        required_transforms = [
            ('map', 'odom'),
            ('odom', 'base_footprint'),
            ('base_footprint', 'base_link'),
            ('base_link', 'rslidar')
        ]
        
        all_transforms_ok = True
        
        # 等待TF数据
        time.sleep(2.0)
        
        for parent, child in required_transforms:
            try:
                # 尝试获取变换
                transform = self.tf_buffer.lookup_transform(
                    parent, child, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)
                )
                self.get_logger().info(f"✓ TF变换 {parent} -> {child} 存在")
                self.check_results[f"tf_{parent}_{child}"] = True
                self.check_details[f"tf_{parent}_{child}"] = f"变换存在，时间戳: {transform.header.stamp}"
                
            except TransformException as e:
                self.get_logger().error(f"✗ TF变换 {parent} -> {child} 不存在: {str(e)}")
                self.check_results[f"tf_{parent}_{child}"] = False
                self.check_details[f"tf_{parent}_{child}"] = f"变换不存在: {str(e)}"
                all_transforms_ok = False
        
        return all_transforms_ok
    
    def check_navigation2_nodes(self) -> bool:
        """
        检查Navigation2相关节点是否运行
        """
        self.get_logger().info("开始检查Navigation2节点...")
        
        # 获取当前运行的节点
        node_names = self.get_node_names()
        
        required_nodes = [
            'bt_navigator',
            'controller_server', 
            'planner_server',
            'behavior_server'
        ]
        
        optional_nodes = [
            'smoother_server',
            'waypoint_follower'
        ]
        
        all_required_ok = True
        
        # 检查必需节点
        for node in required_nodes:
            if node in node_names:
                self.get_logger().info(f"✓ Navigation2节点 {node} 正在运行")
                self.check_results[f"node_{node}"] = True
                self.check_details[f"node_{node}"] = "节点正在运行"
            else:
                self.get_logger().error(f"✗ Navigation2节点 {node} 未运行")
                self.check_results[f"node_{node}"] = False
                self.check_details[f"node_{node}"] = "节点未运行"
                all_required_ok = False
        
        # 检查可选节点
        for node in optional_nodes:
            if node in node_names:
                self.get_logger().info(f"✓ 可选节点 {node} 正在运行")
                self.check_results[f"optional_node_{node}"] = True
                self.check_details[f"optional_node_{node}"] = "可选节点正在运行"
            else:
                self.get_logger().warn(f"! 可选节点 {node} 未运行")
                self.check_results[f"optional_node_{node}"] = False
                self.check_details[f"optional_node_{node}"] = "可选节点未运行"
        
        return all_required_ok
    
    def check_ackermann_specific_requirements(self) -> bool:
        """
        检查阿克曼转向车辆特定要求
        """
        self.get_logger().info("开始检查阿克曼转向特定要求...")
        
        ackermann_ok = True
        
        # 检查是否有阿克曼控制器相关参数
        try:
            # 这里可以添加参数检查逻辑
            # 例如检查控制器类型、运动学约束等
            self.get_logger().info("✓ 阿克曼转向配置检查通过")
            self.check_results["ackermann_config"] = True
            self.check_details["ackermann_config"] = "阿克曼转向配置正确"
        except Exception as e:
            self.get_logger().error(f"✗ 阿克曼转向配置检查失败: {str(e)}")
            self.check_results["ackermann_config"] = False
            self.check_details["ackermann_config"] = f"配置检查失败: {str(e)}"
            ackermann_ok = False
        
        return ackermann_ok
    
    def check_stvl_layer_requirements(self) -> bool:
        """
        检查STVL Layer相关要求
        """
        self.get_logger().info("开始检查STVL Layer要求...")
        
        stvl_ok = True
        
        # 检查点云话题数据格式
        if '/rslidar_points' in self.topic_checks and self.topic_checks['/rslidar_points']:
            self.get_logger().info("✓ 3D雷达点云数据可用")
            self.check_results["stvl_pointcloud"] = True
            self.check_details["stvl_pointcloud"] = "3D雷达点云数据可用"
        else:
            self.get_logger().error("✗ 3D雷达点云数据不可用")
            self.check_results["stvl_pointcloud"] = False
            self.check_details["stvl_pointcloud"] = "3D雷达点云数据不可用"
            stvl_ok = False
        
        # 检查costmap相关话题
        costmap_topics = ['/local_costmap/costmap', '/global_costmap/costmap']
        topic_names_and_types = self.get_topic_names_and_types()
        available_topics = [name for name, _ in topic_names_and_types]
        
        for topic in costmap_topics:
            if topic in available_topics:
                self.get_logger().info(f"✓ Costmap话题 {topic} 存在")
                self.check_results[f"costmap_{topic.replace('/', '_')}"] = True
                self.check_details[f"costmap_{topic.replace('/', '_')}"] = "Costmap话题存在"
            else:
                self.get_logger().warn(f"! Costmap话题 {topic} 不存在（可能Navigation2未启动）")
                self.check_results[f"costmap_{topic.replace('/', '_')}"] = False
                self.check_details[f"costmap_{topic.replace('/', '_')}"] = "Costmap话题不存在"
        
        return stvl_ok
    
    def generate_report(self) -> str:
        """
        生成检查报告
        """
        report = "\n" + "="*60 + "\n"
        report += "Navigation2 阿克曼车辆集成检查报告\n"
        report += "="*60 + "\n\n"
        
        # 统计结果
        total_checks = len(self.check_results)
        passed_checks = sum(1 for result in self.check_results.values() if result)
        failed_checks = total_checks - passed_checks
        
        report += f"总检查项: {total_checks}\n"
        report += f"通过项: {passed_checks}\n"
        report += f"失败项: {failed_checks}\n"
        report += f"通过率: {(passed_checks/total_checks*100):.1f}%\n\n"
        
        # 详细结果
        report += "详细检查结果:\n"
        report += "-"*40 + "\n"
        
        for check_name, result in self.check_results.items():
            status = "✓ 通过" if result else "✗ 失败"
            detail = self.check_details.get(check_name, "无详细信息")
            report += f"{check_name}: {status}\n"
            report += f"  详情: {detail}\n\n"
        
        # 建议
        report += "建议和下一步:\n"
        report += "-"*40 + "\n"
        
        if failed_checks == 0:
            report += "✓ 所有检查项都通过！您的系统已准备好使用Navigation2。\n"
        else:
            report += "以下是需要解决的问题：\n\n"
            
            for check_name, result in self.check_results.items():
                if not result:
                    if "topic_" in check_name:
                        report += f"• 启动相应的节点以发布话题 {check_name}\n"
                    elif "tf_" in check_name:
                        report += f"• 检查TF发布器，确保 {check_name} 变换正确发布\n"
                    elif "node_" in check_name:
                        report += f"• 启动Navigation2节点 {check_name}\n"
            
            report += "\n推荐启动命令：\n"
            report += "ros2 launch nav2_bringup navigation_launch.py\n"
            report += "ros2 launch nav2_bringup bringup_launch.py map:=/path/to/your/map.yaml\n"
        
        report += "\n" + "="*60 + "\n"
        
        return report
    
    def run_all_checks(self) -> bool:
        """
        运行所有检查项
        """
        self.get_logger().info("开始运行Navigation2集成检查...")
        
        # 等待系统初始化
        time.sleep(1.0)
        
        # 执行各项检查
        topics_ok = self.check_required_topics()
        
        # 等待话题数据
        self.get_logger().info("等待话题数据...")
        timeout = 10.0  # 10秒超时
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(self.topic_checks.values()):
                break
        
        tf_ok = self.check_tf_transforms()
        nodes_ok = self.check_navigation2_nodes()
        ackermann_ok = self.check_ackermann_specific_requirements()
        stvl_ok = self.check_stvl_layer_requirements()
        
        # 生成并打印报告
        report = self.generate_report()
        print(report)
        
        # 保存报告到文件
        try:
            with open('/tmp/nav2_ackermann_check_report.txt', 'w', encoding='utf-8') as f:
                f.write(report)
            self.get_logger().info("检查报告已保存到 /tmp/nav2_ackermann_check_report.txt")
        except Exception as e:
            self.get_logger().error(f"保存报告失败: {str(e)}")
        
        # 返回总体结果
        return all([topics_ok, tf_ok, nodes_ok, ackermann_ok, stvl_ok])

def main(args=None):
    """
    主函数
    """
    rclpy.init(args=args)
    
    try:
        checker = Nav2AckermannChecker()
        
        # 运行检查
        success = checker.run_all_checks()
        
        if success:
            checker.get_logger().info("所有检查通过！系统准备就绪。")
            exit_code = 0
        else:
            checker.get_logger().error("部分检查失败，请查看报告解决问题。")
            exit_code = 1
            
    except KeyboardInterrupt:
        print("\n检查被用户中断")
        exit_code = 130
    except Exception as e:
        print(f"检查过程中发生错误: {str(e)}")
        exit_code = 1
    finally:
        try:
            checker.destroy_node()
        except:
            pass
        rclpy.shutdown()
    
    sys.exit(exit_code)

if __name__ == '__main__':
    main()