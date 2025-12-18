/**
 * @file global_path_planner.cpp
 * @brief 全局路径规划节点
 * 
 * 核心功能：
 * 1. 从ROS参数服务器加载航点坐标
 * 2. 使用三次样条插值生成平滑的全局参考路径
 * 3. 计算路径的弧长坐标、偏航角和曲率
 * 4. 发布全局路径供局部规划器使用
 * 
 * @author CYUN
 * @date 2025
 */

#include "../include/frenet_optimal_trajectory.hpp"
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>
#include <iostream>

using namespace std;

/**
 * @brief 全局路径规划节点主函数
 * 
 * 该节点负责生成全局参考路径，包括：
 * 1. 从参数服务器加载航点
 * 2. 生成平滑的三次样条路径
 * 3. 计算路径属性（弧长、曲率等）
 * 4. 发布路径数据供局部规划器使用
 */
int main(int argc, char **argv)
{
	// ========== ROS节点初始化 ==========
	ros::init(argc, argv, "global_path_planner");  // 初始化ROS节点
	ros::NodeHandle n;                              // 创建节点句柄

	// ========== 发布器设置 ==========
	// 发布全局参考路径，用于可视化显示和局部规划器使用
	ros::Publisher global_path_pub = n.advertise<nav_msgs::Path>("/global_path", 1, true);
	// 发布路径坐标数据（x, y坐标数组）
	ros::Publisher path_coords_pub = n.advertise<std_msgs::Float64MultiArray>("/global_path_coords", 1, true);
	// 发布路径属性数据（偏航角、曲率、弧长）
	ros::Publisher path_properties_pub = n.advertise<std_msgs::Float64MultiArray>("/global_path_properties", 1, true);

	// ========== 从ROS参数服务器加载航点参数 ==========
	vecD W_X, W_Y;  // 航点坐标数组
	
	// 从参数服务器加载全局路径的航点坐标
	n.getParam("/frenet_planner/waypoints/W_X", W_X);  // 航点x坐标数组
	n.getParam("/frenet_planner/waypoints/W_Y", W_Y);  // 航点y坐标数组

	// 检查航点数据有效性
	if (W_X.empty() || W_Y.empty() || W_X.size() != W_Y.size())
	{
		ROS_ERROR("Invalid waypoint data! Please check /frenet_planner/waypoints/W_X and W_Y parameters.");
		return -1;
	}

	ROS_INFO("Loaded %zu waypoints for global path planning", W_X.size());

	// ========== 全局路径生成 ==========
	vecD rx, ry, ryaw, rk;  // 全局路径的坐标、偏航角和曲率数组
	double ds = 0.1;        // 三次样条插值的步长 (m)

	// 使用航点生成平滑的全局参考路径
	// 通过三次样条插值连接离散航点，生成连续可导的路径
	ROS_INFO("Generating smooth global path using cubic spline interpolation...");
	Spline2D csp = calc_spline_course(W_X, W_Y, rx, ry, ryaw, rk, ds);

	if (rx.empty() || ry.empty())
	{
		ROS_ERROR("Failed to generate global path! Check waypoint configuration.");
		return -1;
	}

	ROS_INFO("Generated global path with %zu points", rx.size());

	// ========== 计算全局路径的弧长坐标 ==========
	// 弧长坐标是Frenet坐标系中纵向位置的基础
	vector<double> global_s(rx.size());
	double s = 0;
	global_s[0] = 0;  // 起点弧长为0
	
	// 累积计算每个路径点的弧长坐标
	for (unsigned int i = 1; i < rx.size(); i++)
	{
		// 计算相邻路径点间的欧几里得距离
		double dis = dist(rx[i], ry[i], rx[i - 1], ry[i - 1]);
		s = s + dis;        // 累积弧长
		global_s[i] = s;    // 存储当前点的弧长坐标
	}

	ROS_INFO("Total path length: %.2f meters", global_s.back());

	// ========== 构建并发布ROS消息 ==========
	
	// 1. 构建全局路径消息
	nav_msgs::Path global_path_msg;
	global_path_msg.header.frame_id = "map";
	global_path_msg.header.stamp = ros::Time::now();
	global_path_msg.poses.resize(rx.size());

	// 填充路径点数据
	for (unsigned int i = 0; i < rx.size(); i++)
	{
		geometry_msgs::PoseStamped pose;
		pose.header.frame_id = "map";
		pose.header.stamp = ros::Time::now();
		pose.pose.position.x = rx[i];
		pose.pose.position.y = ry[i];
		pose.pose.position.z = 0.0;
		
		// 设置姿态（偏航角转换为四元数）
		pose.pose.orientation.x = 0.0;
		pose.pose.orientation.y = 0.0;
		pose.pose.orientation.z = sin(ryaw[i] / 2.0);
		pose.pose.orientation.w = cos(ryaw[i] / 2.0);
		
		global_path_msg.poses[i] = pose;
	}

	// 2. 构建路径坐标数据消息
	std_msgs::Float64MultiArray coords_msg;
	coords_msg.layout.dim.resize(2);
	coords_msg.layout.dim[0].label = "coordinates";
	coords_msg.layout.dim[0].size = 2;  // x, y
	coords_msg.layout.dim[1].label = "points";
	coords_msg.layout.dim[1].size = rx.size();
	coords_msg.layout.data_offset = 0;
	
	// 填充坐标数据：[x1, y1, x2, y2, ...]
	coords_msg.data.resize(rx.size() * 2);
	for (unsigned int i = 0; i < rx.size(); i++)
	{
		coords_msg.data[i * 2] = rx[i];      // x坐标
		coords_msg.data[i * 2 + 1] = ry[i];  // y坐标
	}

	// 3. 构建路径属性数据消息
	std_msgs::Float64MultiArray properties_msg;
	properties_msg.layout.dim.resize(2);
	properties_msg.layout.dim[0].label = "properties";
	properties_msg.layout.dim[0].size = 3;  // yaw, curvature, arc_length
	properties_msg.layout.dim[1].label = "points";
	properties_msg.layout.dim[1].size = rx.size();
	properties_msg.layout.data_offset = 0;
	
	// 填充属性数据：[yaw1, k1, s1, yaw2, k2, s2, ...]
	properties_msg.data.resize(rx.size() * 3);
	for (unsigned int i = 0; i < rx.size(); i++)
	{
		properties_msg.data[i * 3] = ryaw[i];      // 偏航角
		properties_msg.data[i * 3 + 1] = rk[i];    // 曲率
		properties_msg.data[i * 3 + 2] = global_s[i]; // 弧长坐标
	}

	// ========== 发布路径数据 ==========
	ROS_INFO("Publishing global path data...");
	
	// 发布所有消息
	global_path_pub.publish(global_path_msg);
	path_coords_pub.publish(coords_msg);
	path_properties_pub.publish(properties_msg);

	ROS_INFO("Global path planning completed successfully!");
	ROS_INFO("Published topics:");
	ROS_INFO("  - /global_path: Path visualization");
	ROS_INFO("  - /global_path_coords: Path coordinates (x, y)");
	ROS_INFO("  - /global_path_properties: Path properties (yaw, curvature, arc_length)");

	// ========== 保持节点运行 ==========
	// 使用定时器定期重新发布路径数据，确保新启动的节点能够接收到数据
	ros::Timer republish_timer = n.createTimer(ros::Duration(1.0), 
		[&](const ros::TimerEvent&) {
			// 更新时间戳
			global_path_msg.header.stamp = ros::Time::now();
			for (auto& pose : global_path_msg.poses) {
				pose.header.stamp = ros::Time::now();
			}
			
			// 重新发布
			global_path_pub.publish(global_path_msg);
			path_coords_pub.publish(coords_msg);
			path_properties_pub.publish(properties_msg);
		});

	// 保持节点运行
	ros::spin();

	return 0;
}