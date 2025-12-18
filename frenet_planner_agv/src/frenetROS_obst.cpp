/**
 * @file frenetROS_obst.cpp
 * @brief Frenet路径规划算法的ROS接口实现
 * 
 * 核心功能：
 * 1. 从ROS话题获取机器人状态和环境信息
 * 2. 将代价地图转换为障碍物坐标
 * 3. 基于当前状态计算Frenet坐标系初始条件
 * 4. 调用Frenet路径规划算法生成最优轨迹
 * 5. 将规划结果发布为ROS消息供导航使用
 * 
 * @author Frenet Planner Team
 * @date 2024
 */

#include "../include/frenet_optimal_trajectory.hpp"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <time.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_datatypes.h>
#include <utility>
#include <ros/console.h>
#include <vector>
#include <iostream>
#include <cstdio>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

// 可视化功能已移除，不再需要matplotlib-cpp

// 全局计数器，用于调试和性能监控
int cost_count = 0;      ///< 代价地图回调函数调用次数
int footprint_count = 0; ///< 机器人轮廓回调函数调用次数
int odom_count = 0;      ///< 里程计回调函数调用次数

/**
 * @brief 代价地图回调函数
 * 
 * 处理从ROS导航栈接收到的代价地图数据，将占用栅格转换为障碍物坐标点。
 * 该函数是路径规划中障碍物检测的核心，将二维栅格地图中的障碍物
 * 转换为世界坐标系下的点集合，供碰撞检测使用。
 * 
 * 处理流程：
 * 1. 遍历代价地图的每个栅格单元
 * 2. 识别占用值大于0的栅格（障碍物）
 * 3. 将栅格坐标转换为世界坐标
 * 4. 存储障碍物坐标点供路径规划使用
 * 
 * @param occupancy_grid 代价地图消息指针，包含栅格数据和地图信息
 */
void costmap_callback(const nav_msgs::OccupancyGrid::ConstPtr &occupancy_grid)
{
	cost_count++;  // 增加回调计数器
	unsigned int height, width;
	::cmap = *occupancy_grid;  // 保存代价地图副本
	
	// 清空之前的障碍物坐标
	ob_x.clear();
	ob_y.clear();
	
	// 获取地图原点信息，用于坐标转换
	geometry_msgs::Pose origin = occupancy_grid->info.origin;

	// 临时存储障碍物坐标的容器
	vector<pair<double, double>> ob1;

	// 使用OpenMP并行处理提高性能，遍历整个代价地图
#pragma omp parallel for collapse(2)
	for (width = 0; width < occupancy_grid->info.width; ++width)
	{
		for (height = 0; height < occupancy_grid->info.height; ++height)
		{
			// 检查栅格是否被占用（值大于0表示有障碍物）
			if (occupancy_grid->data[height * occupancy_grid->info.width + width] > 0)
			{
				// 临界区保护，确保多线程安全
#pragma omp critical
				{
					// 将栅格坐标转换为世界坐标
					// 公式：世界坐标 = 栅格索引 × 分辨率 + 栅格中心偏移 + 地图原点
					double world_x = width * occupancy_grid->info.resolution + 
					                 occupancy_grid->info.resolution / 2 + 
					                 origin.position.x;
					double world_y = height * occupancy_grid->info.resolution + 
					                 occupancy_grid->info.resolution / 2 + 
					                 origin.position.y;
					ob1.emplace_back(world_x, world_y);
				}
			}
		}
	}

	// 对障碍物坐标进行排序，便于后续搜索和处理
	sort(ob1.begin(), ob1.end());
	
	// 调整全局障碍物坐标数组大小
	ob_x.resize(ob1.size());
	ob_y.resize(ob1.size());

	// 将障碍物坐标复制到全局数组中
	for (long i = 0; i < ob1.size(); i++)
	{
		ob_x[i] = ob1[i].first;   // x坐标
		ob_y[i] = ob1[i].second;  // y坐标
	}
}

/**
 * @brief 机器人轮廓回调函数
 * 
 * 接收并存储机器人的几何轮廓信息，用于碰撞检测。
 * 轮廓定义了机器人的外形边界，是精确碰撞检测的基础。
 * 
 * @param p 机器人轮廓消息指针，包含多边形顶点坐标
 */
void footprint_callback(const geometry_msgs::PolygonStampedConstPtr &p)
{
	::footprint = *p;  // 保存机器人轮廓到全局变量
	footprint_count++;  // 增加轮廓消息计数器
}

/**
 * @brief 里程计回调函数
 * 
 * 接收并存储机器人的位姿和速度信息，包括位置、方向和线速度、角速度。
 * 这些信息是路径规划算法计算初始条件的重要输入。
 * 
 * @param msg 里程计消息指针，包含机器人的完整状态信息
 */
void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
	::odom = *msg;  // 保存里程计数据到全局变量
	odom_count++;   // 增加里程计消息计数器
}

/**
 * @brief 在全局路径中查找距离机器人最近的点
 * 
 * 该函数用于确定机器人在全局参考路径上的投影点，这是Frenet坐标系
 * 转换的关键步骤。通过找到最近点，可以计算机器人相对于参考路径的
 * 横向偏移和纵向位置。
 * 
 * @param global_x 全局路径x坐标数组
 * @param global_y 全局路径y坐标数组
 * @param min_x 输出：最近点的x坐标
 * @param min_y 输出：最近点的y坐标
 * @param min_dis 输出：最小距离
 * @param min_id 输出：最近点在路径中的索引
 * @param flag 标志位：0-使用里程计位置，1-使用路径点位置
 * @param path Frenet路径对象（当flag=1时使用）
 */
void find_nearest_in_global_path(vecD &global_x, vecD &global_y, double &min_x, double &min_y,
								 double &min_dis, int &min_id, int flag, FrenetPath &path)
{
	double bot_x, bot_y;

	// 根据标志位选择机器人位置来源
	if (flag == 0)
	{
		// 使用里程计提供的当前位置
		bot_x = odom.pose.pose.position.x;
		bot_y = odom.pose.pose.position.y;
	}
	else
	{
		// 使用路径中的预测位置（第二个点）
		bot_x = path.get_x()[1];
		bot_y = path.get_y()[1];
	}

	// 初始化最小距离为最大浮点数
	min_dis = FLT_MAX;

	// 遍历全局路径上的所有点，寻找最近点
	for (unsigned int i = 0; i < global_x.size(); i++)
	{
		// 计算机器人位置到路径点的欧几里得距离
		double dis = dist(global_x[i], global_y[i], bot_x, bot_y);
		
		// 更新最近点信息
		if (dis < min_dis)
		{
			min_dis = dis;        // 最小距离
			min_x = global_x[i];  // 最近点x坐标
			min_y = global_y[i];  // 最近点y坐标
			min_id = i;           // 最近点索引
		}
	}
}

/**
 * @brief 从四元数获取机器人的偏航角
 * 
 * 将里程计中的四元数姿态信息转换为欧拉角，提取偏航角（yaw）。
 * 偏航角表示机器人在水平面内的朝向，是路径规划中的重要参数。
 * 
 * @return 机器人的偏航角（弧度）
 */
inline double get_bot_yaw()
{
	// 从里程计消息中提取姿态信息
	geometry_msgs::Pose p = odom.pose.pose;
	
	// 构造四元数对象
	tf::Quaternion q(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
	
	// 构造旋转矩阵
	tf::Matrix3x3 m(q);
	
	// 转换为欧拉角（滚转、俯仰、偏航）
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	
	return yaw;  // 返回偏航角
}

/**
 * @brief 计算Frenet坐标系的初始条件
 * 
 * 基于机器人当前状态和全局参考路径，计算Frenet坐标系下的初始条件。
 * 这些初始条件是路径采样和轨迹生成的基础，包括：
 * - 纵向位置s0：机器人在参考路径上的投影位置
 * - 横向位置c_d：机器人相对于参考路径的横向偏移
 * - 横向速度c_d_d：横向偏移的变化率
 * - 纵向速度c_speed：沿参考路径方向的速度分量
 * 
 * 算法流程：
 * 1. 找到全局路径上距离机器人最近的点
 * 2. 计算横向偏移距离和方向
 * 3. 基于速度和航向角计算Frenet坐标系下的速度分量
 * 4. 考虑路径曲率对速度的影响
 * 
 * @param csp 三次样条插值对象，用于路径插值和导数计算
 * @param global_s 全局路径的弧长坐标数组
 * @param global_x 全局路径的x坐标数组
 * @param global_y 全局路径的y坐标数组
 * @param global_R 全局路径的曲率数组
 * @param global_yaw 全局路径的航向角数组
 * @param s0 输出：纵向位置（弧长坐标）
 * @param c_speed 输出：纵向速度
 * @param c_d 输出：横向位置偏移
 * @param c_d_d 输出：横向速度
 * @param c_d_dd 输出：横向加速度
 * @param bot_yaw 输出：机器人当前偏航角
 * @param path Frenet路径对象（用于最近点搜索）
 * @return 最近点在全局路径中的索引
 */
int initial_conditions_new(Spline2D &csp, vecD &global_s, vecD &global_x, vecD &global_y, vecD &global_R, vecD &global_yaw, double &s0, double &c_speed, double &c_d, double &c_d_d,double &c_d_dd, double &bot_yaw, FrenetPath &path)
{
	// 获取机器人当前线速度分量
	double vx = odom.twist.twist.linear.x;
	double vy = odom.twist.twist.linear.y;
	// 计算速度模长
	double v = sqrt(vx * vx + vy * vy);
	
	double min_x, min_y;
	int min_id;

	// 在全局路径中找到距离机器人最近的点，获取横向距离
	find_nearest_in_global_path(global_x, global_y, min_x, min_y, c_d, min_id, 0, path);

	// 确定横向偏移的符号（左侧为正，右侧为负）
	// 使用叉积判断机器人相对于路径的左右位置关系
	pair<double, double> vec1, vec2;
	// vec1: 从路径点指向机器人的向量
	vec1.first = odom.pose.pose.position.x - global_x[min_id];
	vec1.second = odom.pose.pose.position.y - global_y[min_id];
	// vec2: 路径切线方向向量（指向下一个点）
	vec2.first = global_x[min_id] - global_x[min_id + 1];
	vec2.second = global_y[min_id] - global_y[min_id + 1];
	// 计算2D叉积，判断机器人在路径的左侧还是右侧
	double curl2D = vec1.first * vec2.second - vec2.first * vec1.second;
	if (curl2D < 0)
		c_d *= -1;  // 右侧为负值
	
	// 设置纵向位置为最近点的弧长坐标
	s0 = global_s[min_id];
	
	// 获取机器人当前偏航角
	bot_yaw = get_bot_yaw();
	
	// 获取全局路径在最近点处的切线方向
	double g_path_yaw = global_yaw[min_id];
	
	// 计算机器人航向与路径切线方向的夹角
	double delta_theta = bot_yaw - g_path_yaw;
	
	// 计算横向速度（Frenet坐标系中d的一阶导数）
	// 基于运动学关系：横向速度 = 总速度 × sin(航向角差)
	c_d_d = v * sin(delta_theta); // Equation 5
	
	// 获取路径在最近点处的曲率
	double k_r = global_R[min_id];
	
	// 计算纵向速度（Frenet坐标系中s的一阶导数）
	// 考虑曲率和横向偏移的影响：s_dot = v*cos(Δθ) / (1 - κ*d)
	c_speed = v * cos(delta_theta) / (1 - k_r * c_d); // s_dot (Equation 7)
	
	// 横向加速度初始化为0（后续可根据需要更新）
	c_d_dd = 0;										  // For the time being. Need to be updated
	
	return min_id;  // 返回最近点索引
}

/**
 * @brief 发布Frenet路径到ROS话题
 * 
 * 将Frenet规划器生成的最优路径转换为ROS标准的nav_msgs::Path消息格式，
 * 并发布到指定话题。路径包含位置和姿态信息，可用于可视化和导航控制。
 * 
 * 转换过程：
 * 1. 创建路径消息头，设置坐标系和时间戳
 * 2. 遍历路径上的每个点，转换为PoseStamped格式
 * 3. 将偏航角转换为四元数表示姿态
 * 4. 发布完整的路径消息
 * 
 * @param path_msg 输出的ROS路径消息
 * @param path Frenet路径对象，包含x、y坐标序列
 * @param rk 路径曲率数组
 * @param ryaw 路径偏航角数组
 * @param c_speed 当前纵向速度
 * @param c_d 当前横向偏移
 * @param c_d_d 当前横向速度
 */
void publishPath(nav_msgs::Path &path_msg, FrenetPath &path, vecD &rk, vecD &ryaw, double &c_speed,
				 double &c_d, double &c_d_d)
{
	geometry_msgs::PoseStamped loc;
	double delta_theta, yaw;
	vecD x_vec = path.get_x();
	vecD y_vec = path.get_y();
	
	// 遍历路径上的每个点，转换为ROS消息格式
	for (unsigned int i = 0; i < path.get_x().size(); i++)
	{
		// 设置位置信息
		loc.pose.position.x = x_vec[i];     // x坐标
		loc.pose.position.y = y_vec[i];     // y坐标
		
		// 计算机器人在该点的姿态角
		// delta_theta是相对于参考路径的角度偏差
		delta_theta = atan(c_d_d / ((1 - rk[i] * c_d) * c_speed));
		// 最终偏航角 = 角度偏差 + 参考路径偏航角
		yaw = delta_theta + ryaw[i];
		
		// 将偏航角转换为四元数表示姿态
		tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, yaw); // roll, pitch = 0
		q.normalize();  // 归一化四元数
		quaternionTFToMsg(q, loc.pose.orientation);
		
		// 将位姿添加到路径消息中
		path_msg.poses.push_back(loc);
	}
}

/**
 * @brief 发布采样路径的MarkerArray可视化
 * 
 * 将Frenet规划器生成的所有采样路径转换为MarkerArray消息格式，
 * 用于在RViz中可视化显示所有候选轨迹。每条路径用不同颜色的线条表示。
 * 
 * @param sampled_paths_pub MarkerArray发布器
 * @param fplist 所有采样路径的集合
 */
void publishSampledPaths(ros::Publisher &sampled_paths_pub, vector<FrenetPath> &fplist)
{
	visualization_msgs::MarkerArray marker_array;
	
	// 清除之前的标记
	visualization_msgs::Marker clear_marker;
	clear_marker.header.frame_id = "map";
	clear_marker.header.stamp = ros::Time::now();
	clear_marker.ns = "sampled_paths";
	clear_marker.action = visualization_msgs::Marker::DELETEALL;
	marker_array.markers.push_back(clear_marker);
	
	// 为每条采样路径创建线条标记
	for (size_t i = 0; i < fplist.size() && i < 50; ++i) // 限制显示路径数量避免过载
	{
		// 获取路径点数据
		vecD x_vec = fplist[i].get_x();
		vecD y_vec = fplist[i].get_y();
		vecD s_vec = fplist[i].get_s();
		vecD d_vec = fplist[i].get_d();
		
		// 添加详细调试信息
		ROS_INFO("Path %zu debug info:", i);
		ROS_INFO("  s.size(): %zu, d.size(): %zu", s_vec.size(), d_vec.size());
		ROS_INFO("  x.size(): %zu, y.size(): %zu", x_vec.size(), y_vec.size());
		if (!s_vec.empty()) {
			ROS_INFO("  s range: [%.3f, %.3f]", s_vec.front(), s_vec.back());
		}
		if (!d_vec.empty()) {
			ROS_INFO("  d range: [%.3f, %.3f]", d_vec.front(), d_vec.back());
		}
		if (!x_vec.empty() && !y_vec.empty()) {
			ROS_INFO("  First point: (%.3f, %.3f)", x_vec[0], y_vec[0]);
			if (x_vec.size() > 1) {
				ROS_INFO("  Last point: (%.3f, %.3f)", x_vec.back(), y_vec.back());
			}
		}
		
		// 检查路径点是否为空或少于2个点
		if (x_vec.empty() || y_vec.empty() || x_vec.size() < 2 || y_vec.size() < 2) {
			ROS_WARN("Skipping path %zu: insufficient points (x: %zu, y: %zu)", i, x_vec.size(), y_vec.size());
			continue;
		}
		
		visualization_msgs::Marker line_marker;
		line_marker.header.frame_id = "map";
		line_marker.header.stamp = ros::Time::now();
		line_marker.ns = "sampled_paths";
		line_marker.id = i;
		line_marker.type = visualization_msgs::Marker::LINE_STRIP;
		line_marker.action = visualization_msgs::Marker::ADD;
		
		// 初始化四元数为单位四元数（修复"Uninitialized quaternion"错误）
		line_marker.pose.orientation.x = 0.0;
		line_marker.pose.orientation.y = 0.0;
		line_marker.pose.orientation.z = 0.0;
		line_marker.pose.orientation.w = 1.0;
		
		// 设置线条属性
		line_marker.scale.x = 0.05; // 线条宽度
		
		// 根据路径索引设置不同颜色
		if (i == 0) {
			// 最优路径用绿色
			line_marker.color.r = 0.0;
			line_marker.color.g = 1.0;
			line_marker.color.b = 0.0;
			line_marker.color.a = 0.8;
			line_marker.scale.x = 0.1; // 最优路径更粗
		} else {
			// 其他路径用半透明蓝色
			line_marker.color.r = 0.0;
			line_marker.color.g = 0.5;
			line_marker.color.b = 1.0;
			line_marker.color.a = 0.3;
		}
		
		// 添加路径点
		for (size_t j = 0; j < x_vec.size(); ++j)
		{
			geometry_msgs::Point point;
			point.x = x_vec[j];
			point.y = y_vec[j];
			point.z = 0.0;
			line_marker.points.push_back(point);
		}
		
		marker_array.markers.push_back(line_marker);
	}
	
	// 发布MarkerArray
	sampled_paths_pub.publish(marker_array);
}

/**
 * @brief 主函数 - Frenet路径规划器的ROS节点入口
 * 
 * 该函数是整个Frenet路径规划系统的核心控制程序，负责：
 * 1. 初始化ROS节点和通信接口
 * 2. 加载规划参数配置
 * 3. 生成全局参考路径
 * 4. 执行主控制循环，进行实时路径规划
 * 5. 发布控制指令和可视化信息
 * 
 * 系统架构：
 * - 订阅：代价地图、机器人轮廓、里程计信息
 * - 发布：规划路径、速度控制指令
 * - 核心算法：基于Frenet坐标系的最优轨迹规划
 * 
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return 程序退出状态码
 */
int main(int argc, char **argv)
{
	bool gotOdom = false;

	// ========== ROS节点初始化 ==========
	ros::init(argc, argv, "frenet_planner");  // 初始化ROS节点
	ros::NodeHandle n;                        // 创建节点句柄

	// ========== 发布器设置 ==========
	// 发布规划生成的Frenet路径，用于可视化和导航控制
	ros::Publisher frenet_path = n.advertise<nav_msgs::Path>("/frenet_path", 1);
	// 发布全局参考路径，用于可视化显示
	ros::Publisher global_path = n.advertise<nav_msgs::Path>("/global_path", 1);
	// 发布速度控制指令，控制机器人运动
	ros::Publisher target_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	// 发布采样路径的MarkerArray可视化
	ros::Publisher sampled_paths_pub = n.advertise<visualization_msgs::MarkerArray>("/frenet_sampled_paths", 1);

	// ========== 订阅器设置 ==========
	
	// 订阅里程计信息，获取机器人当前位姿和速度
	ros::Subscriber odom_sub = n.subscribe("/base_pose_ground_truth", 10, odom_callback);
	// 订阅机器人轮廓信息，用于精确碰撞检测
	ros::Subscriber footprint_sub = n.subscribe<geometry_msgs::PolygonStamped>("/move_base/local_costmap/footprint", 10, footprint_callback);
	// 订阅局部代价地图，用于障碍物检测和避障
	ros::Subscriber costmap_sub = n.subscribe<nav_msgs::OccupancyGrid>(
		"/move_base/local_costmap/costmap", 10, costmap_callback);

	// ========== 从ROS参数服务器加载规划参数 ==========
	// 运动约束参数
	n.getParam("/frenet_planner/path/max_speed", MAX_SPEED);           // 最大速度限制
	n.getParam("/frenet_planner/path/max_accel", MAX_ACCEL);           // 最大加速度限制
	n.getParam("/frenet_planner/path/max_curvature", MAX_CURVATURE);   // 最大曲率限制
	n.getParam("/frenet_planner/path/max_road_width", MAX_ROAD_WIDTH); // 最大道路宽度
	n.getParam("/frenet_planner/path/d_road_w", D_ROAD_W);             // 横向采样间隔
	n.getParam("/frenet_planner/path/dt", DT);                         // 时间步长
	n.getParam("/frenet_planner/path/maxt", MAXT);                     // 最大预测时间
	n.getParam("/frenet_planner/path/mint", MINT);                     // 最小预测时间
	n.getParam("/frenet_planner/path/target_speed", TARGET_SPEED);     // 目标速度
	n.getParam("/frenet_planner/path/d_t_s", D_T_S);                   // 速度采样间隔
	n.getParam("/frenet_planner/path/n_s_sample", N_S_SAMPLE);         // 纵向采样数量
	n.getParam("/frenet_planner/path/robot_radius", ROBOT_RADIUS);     // 机器人半径
	n.getParam("/frenet_planner/path/max_lat_vel", MAX_LAT_VEL);       // 最大横向速度
	n.getParam("/frenet_planner/path/min_lat_vel", MIN_LAT_VEL);       // 最小横向速度
	n.getParam("/frenet_planner/path/d_d_ns", D_D_NS);                 // 横向速度采样间隔
	n.getParam("/frenet_planner/path/max_shift_d", MAX_SHIFT_D);       // 最大横向偏移

	// 成本函数权重参数
	n.getParam("/frenet_planner/cost/kj", KJ);       // 加加速度权重
	n.getParam("/frenet_planner/cost/kt", KT);       // 时间权重
	n.getParam("/frenet_planner/cost/kd", KD);       // 横向偏移权重
	n.getParam("/frenet_planner/cost/kd_v", KD_V);   // 横向速度权重
	n.getParam("/frenet_planner/cost/klon", KLON);   // 纵向轨迹权重
	n.getParam("/frenet_planner/cost/klat", KLAT);   // 横向轨迹权重
	n.getParam("/frenet_planner/path/Rate", RATE);   // 循环执行频率

	// ========== 全局路径订阅相关变量 ==========
	bool global_path_received = false;
	bool global_coords_received = false;
	bool global_properties_received = false;
	
	// 全局路径数据存储
	vecD rx, ry, ryaw, rk;
	vector<double> global_s;

	// ========== 全局路径订阅回调函数 ==========
	// 订阅全局路径坐标数据
	ros::Subscriber global_coords_sub = n.subscribe<std_msgs::Float64MultiArray>("/global_path_coords", 1, 
		[&](const std_msgs::Float64MultiArray::ConstPtr& msg) {
			if (msg->data.size() % 2 != 0) {
				ROS_WARN("Invalid global path coordinates data size: %zu", msg->data.size());
				return;
			}
			
			size_t num_points = msg->data.size() / 2;
			rx.resize(num_points);
			ry.resize(num_points);
			
			for (size_t i = 0; i < num_points; i++) {
				rx[i] = msg->data[i * 2];
				ry[i] = msg->data[i * 2 + 1];
			}
			
			global_coords_received = true;
			// ROS_INFO("Received global path coordinates: %zu points", num_points);
		});
	
	// 订阅全局路径属性数据
	ros::Subscriber global_properties_sub = n.subscribe<std_msgs::Float64MultiArray>("/global_path_properties", 1,
		[&](const std_msgs::Float64MultiArray::ConstPtr& msg) {
			if (msg->data.size() % 3 != 0) {
				ROS_WARN("Invalid global path properties data size: %zu", msg->data.size());
				return;
			}
			
			size_t num_points = msg->data.size() / 3;
			ryaw.resize(num_points);
			rk.resize(num_points);
			global_s.resize(num_points);
			
			for (size_t i = 0; i < num_points; i++) {
				ryaw[i] = msg->data[i * 3];
				rk[i] = msg->data[i * 3 + 1];
				global_s[i] = msg->data[i * 3 + 2];
			}
			
			global_properties_received = true;
			// ROS_INFO("Received global path properties: %zu points", num_points);
		});
	
	// 订阅全局路径可视化数据
	ros::Subscriber global_path_sub = n.subscribe<nav_msgs::Path>("/global_path", 1,
		[&](const nav_msgs::Path::ConstPtr& msg) {
			global_path_received = true;
			// ROS_INFO("Received global path visualization: %zu poses", msg->poses.size());
		});

	double bot_yaw, bot_v;  // 机器人偏航角和速度
	Spline2D csp;  // 三次样条插值对象

	// ========== 路径规划变量初始化 ==========
	FrenetPath path;  // 当前规划的最优路径
	FrenetPath lp;    // 上一次规划的路径（用于连续性）
	
	// Frenet坐标系初始条件变量
	double s0, c_d, c_d_d, c_d_dd, c_speed;
	unsigned int ctr = 0, i;  // 循环计数器和索引
	
	// ========== 等待所有必要话题订阅成功 ==========
	ROS_INFO("Waiting for all required topics...");
	ROS_INFO("Required topics:");
	ROS_INFO("  - /global_path (visualization)");
	ROS_INFO("  - /global_path_coords (coordinates)");
	ROS_INFO("  - /global_path_properties (properties)");
	ROS_INFO("  - /move_base/local_costmap/costmap");
	ROS_INFO("  - /move_base/local_costmap/footprint");
	ROS_INFO("  - /base_pose_ground_truth");
	
	// 等待所有话题订阅成功
	ros::Rate wait_rate(1); // 1Hz检查频率
	while (ros::ok() && (!global_path_received || !global_coords_received || 
	                     !global_properties_received || cost_count == 0 || 
	                     footprint_count == 0 || odom_count == 0))
	{
		ros::spinOnce();
		
		// 显示订阅状态
		ROS_INFO_THROTTLE(5, "Topic subscription status:");
		ROS_INFO_THROTTLE(5, "  Global path: %s", global_path_received ? "OK" : "NO");
		ROS_INFO_THROTTLE(5, "  Global coords: %s", global_coords_received ? "OK" : "NO");
		ROS_INFO_THROTTLE(5, "  Global properties: %s", global_properties_received ? "OK" : "NO");
		ROS_INFO_THROTTLE(5, "  Costmap: %s", (cost_count > 0) ? "OK" : "NO");
		ROS_INFO_THROTTLE(5, "  Footprint: %s", (footprint_count > 0) ? "OK" : "NO");
		ROS_INFO_THROTTLE(5, "  Odometry: %s", (odom_count > 0) ? "OK" : "NO");
		
		wait_rate.sleep();
	}
	
	ROS_INFO("All required topics are now available! Starting local planning...");
	
	// 创建三次样条插值对象
	csp = Spline2D(rx, ry);
	
	// 设置目标弧长（路径终点）
	s_dest = global_s.back();
	bool run_frenet = true;  // 控制Frenet规划器运行状态
	bool endpoint_reached_printed = false;  // 终点到达打印状态标志

	// 创建频率控制对象
	ros::Rate rate(RATE);  // 使用参数配置的频率

	// ========== 主控制循环 ==========
	// 该循环是整个系统的核心，持续进行路径规划和控制
	while (ros::ok())
	{
		// 记录当前循环开始时间，用于性能分析
		double startTime0 = omp_get_wtime();

		int min_id = 0;  // 最近路径点的索引

		// ========== 计算Frenet坐标系初始条件 ==========
		// 基于当前里程计信息，计算机器人在Frenet坐标系下的状态
		// 这是路径规划的起始条件，包括位置、速度和加速度
		min_id = initial_conditions_new(csp, global_s, rx, ry, rk, ryaw, s0, c_speed, c_d, c_d_d, c_d_dd, bot_yaw, path);


		// ========== 目标到达检测和速度调节 ==========
		// 当接近目标点时，逐步减速直至停止
		if (abs(s_dest - s0) <= 15)  // 距离目标15米内
		{
			STOP_CAR = true;      // 设置停车标志
			TARGET_SPEED = 0;     // 目标速度设为0
			// 只在首次到达终点时打印一次
			if (!endpoint_reached_printed) {
				cout << "到达终点，开始停车\n";     // 输出停车信息
				endpoint_reached_printed = true;
			}
		}
		else if (endpoint_reached_printed)  // 如果之前到达过终点但现在离开了
		{
			// 重置终点到达状态，允许下次到达时重新打印
			endpoint_reached_printed = false;
		}
		if (abs(s_dest - s0) <= 5)   // 距离目标5米内
		{
			c_speed /= 2;         // 当前速度减半，实现平滑减速
		}

		// ========== 执行Frenet最优路径规划 ==========
		// 这是系统的核心算法，执行以下步骤：
		// 1. 在Frenet坐标系中采样多条候选轨迹
		// 2. 将轨迹转换回笛卡尔坐标系
		// 3. 检查每条轨迹的碰撞和约束条件
		// 4. 计算每条轨迹的综合成本
		// 5. 返回成本最低的最优轨迹
		path = frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, lp, bot_yaw);

		// 获取所有采样路径并发布可视化
		Fplist* current_fplist = get_current_fplist();
		if (current_fplist != nullptr && !current_fplist->fplist_lat.empty()) {
			publishSampledPaths(sampled_paths_pub, current_fplist->fplist_lat);
		}

		// 保存当前路径作为下一次规划的参考（保持连续性）
		lp = path;
		
		// ========== 准备ROS消息 ==========
		nav_msgs::Path path_msg;        // Frenet规划路径消息
		nav_msgs::Path global_path_msg; // 全局参考路径消息

		// 设置消息头，指定坐标系为地图坐标系
		path_msg.header.frame_id = "map";
		global_path_msg.header.frame_id = "map";
		global_path_msg.poses.resize(rx.size());  // 预分配空间

		// ========== 构建全局路径消息 ==========
		// 将全局参考路径转换为ROS消息格式，用于可视化
		for (i = 0; i < rx.size(); i++)
		{
			geometry_msgs::PoseStamped loc;
			loc.pose.position.x = rx[i];  // 全局路径x坐标
			loc.pose.position.y = ry[i];  // 全局路径y坐标
			global_path_msg.poses[i] = loc;
		}

		// ========== 构建Frenet规划路径消息 ==========
		// 将Frenet规划的最优轨迹转换为ROS消息格式
		// 包括坐标变换和姿态计算
		publishPath(path_msg, path, rk, ryaw, c_d, c_speed, c_d_d);

		// ========== 计算机器人实际速度 ==========
		// 定义Lambda函数计算机器人在Frenet坐标系下的实际速度
		// 使用路径中点的状态进行计算（需要进一步验证中点选择的合理性）
		auto calc_bot_v = [min_id, rk](vecD d, vecD s_d, vecD d_d)
		{
			// 基于Frenet坐标系运动学方程计算实际速度
			// v = sqrt((1-κ*d)²*s_dot² + d_dot²)
			// 其中：κ为曲率，d为横向偏移，s_dot为纵向速度，d_dot为横向速度
			return sqrt(pow(1 - rk[min_id] * d[d.size() / 2], 2) * pow(s_d[s_d.size() / 2], 2) +
						pow(d_d[d_d.size() / 2], 2));
		};

		// ========== 确定下一步的目标速度 ==========
		// 根据路径规划结果计算机器人应该执行的速度指令
		if (path.get_d().size() <= 1 || path.get_s_d().size() <= 1 || path.get_d_d().size() <= 1)
		{
			// 路径数据不足时，使用当前状态计算速度
			bot_v = sqrt(pow(1 - rk[min_id] * c_d, 2) * pow(c_speed, 2) + pow(c_d_d, 2));
		}
		else
		{
			if (STOP_CAR)
			{
				// 停车模式：使用中点状态计算减速速度
				bot_v = calc_bot_v(path.get_d(), path.get_s_d(), path.get_d_d());
			}
			else
			{
				// 正常行驶模式：使用路径第二个点的状态计算速度
				// 选择第二个点是为了避免当前点的不稳定性
				bot_v = sqrt(pow(1 - rk[min_id] * path.get_d()[1], 2) * pow(path.get_s_d()[1], 2) +
							 pow(path.get_d_d()[1], 2));
			}
		}

		// ========== 构建并发布控制指令 ==========
		// 创建速度控制消息
		geometry_msgs::Twist vel;
		vel.linear.x = bot_v;  // 前向线速度
		vel.linear.y = 0;      // 侧向线速度（差分驱动机器人为0）
		vel.linear.z = 0;      // 垂直线速度（2D导航为0）
		
		// 发布所有消息
		frenet_path.publish(path_msg);        // 发布Frenet规划路径
		global_path.publish(global_path_msg); // 发布全局参考路径
		target_vel.publish(vel);              // 发布速度控制指令
		
		ctr++;  // 增加循环计数器

		// ========== 性能监控 ==========
		// 计算当前循环的执行时间和频率
		double endTime0 = omp_get_wtime();
		double frequency = 1 / (endTime0 - startTime0);  // 计算执行频率
		// cout << "frequency: " << frequency << endl;
		// 处理ROS回调函数
		ros::spinOnce();
		
		// 控制循环执行频率
		rate.sleep();
	}
	
	return 0;  // 程序正常退出
}
