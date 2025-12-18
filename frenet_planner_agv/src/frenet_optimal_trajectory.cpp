/**
 * @file frenet_optimal_trajectory.cpp
 * @brief Frenet坐标系最优轨迹规划算法实现
 * 
 * 算法核心思想：
 * 1. 在Frenet坐标系下分别生成横向和纵向轨迹
 * 2. 通过五次多项式拟合生成平滑轨迹
 * 3. 将Frenet轨迹转换为全局坐标系
 * 4. 进行碰撞检测和约束检查
 * 5. 基于成本函数选择最优轨迹
 * 
 * @author Frenet规划算法团队
 * @date 2024
 * @version 1.0
 */

#include "../include/frenet_optimal_trajectory.hpp"
#include "../include/polynomials.hpp"
#include <ros/console.h>
#include <bits/stdc++.h>
#include <algorithm>
#include <vector>

// Removed matplotlib-cpp namespace alias (visualization no longer needed)

// 全局变量，用于存储当前的Fplist对象
Fplist* current_fplist = nullptr;

/**
 * @brief 获取当前Fplist对象指针
 * @return 当前Fplist对象的指针，用于访问所有采样路径
 */
Fplist* get_current_fplist() {
    return current_fplist;
}

/**
 * @brief 将Frenet坐标轨迹转换为全局坐标轨迹
 * @param fp Frenet坐标轨迹对象
 * @param csp 参考路径的三次样条插值器
 * @return 转换后的全局坐标轨迹
 */
FrenetPath calc_global_path(FrenetPath fp, Spline2D csp)
{
	fp.adding_global_path(csp);
	return fp;
}

/**
 * @brief 将Frenet坐标轨迹转换为全局笛卡尔坐标轨迹
 * 
 * 该函数实现Frenet坐标系到全局坐标系的转换：
 * 1. 根据纵向位移s在参考路径上找到对应点
 * 2. 根据横向位移d计算偏移后的全局坐标
 * 3. 计算轨迹的航向角和曲率
 * 
 * 转换公式：
 * x = ix - d * sin(iyaw)
 * y = iy + d * cos(iyaw)
 * 
 * @param csp 参考路径的三次样条插值器
 */
void FrenetPath::adding_global_path(Spline2D csp)
{
	int n = s.size();
	x.resize(n);
	y.resize(n);
	
	// 遍历每个轨迹点，进行坐标转换
	for (int i = 0; i < n; i++)
	{
		// 根据纵向位移s获取参考路径上的点坐标
		Eigen::Vector2d pos = csp.calc_postion(s[i]);
		double ix = pos.x();
		double iy = pos.y();
		if (ix == NONE)
		{
			// 如果超出参考路径范围，直接返回
			return;
		}
		// 获取参考路径在该点的航向角
		double iyaw = csp.calc_yaw(s[i]);
		// 根据Frenet坐标转换公式计算全局坐标
		double fx = ix - d[i] * sin(iyaw);  // x坐标
		double fy = iy + d[i] * cos(iyaw);  // y坐标
		x[i] = fx;
		y[i] = fy;
	}
	
	// 计算轨迹的航向角和弧长增量
	yaw.resize(n - 1);
	ds.resize(n - 1);

	for (int i = 0; i < n - 1; i++)
	{
		// 计算相邻点之间的位移
		double dx = x[i + 1] - x[i];
		double dy = y[i + 1] - y[i];
		
		// 计算航向角
		if (abs(dx) > 0.0001)
		{
			yaw[i] = atan2(dy, dx);
		}
		else
		{
			yaw[i] = 0.0;
		}

		// 计算弧长增量
		ds[i] = sqrt(dx * dx + dy * dy);
	}
	
	// 检查轨迹是否超出参考路径边界
	if (s.size() == x.size())
	{
		return;
	}
	
	// 计算轨迹曲率
	c.resize((n - 1) - 1);
	for (int i = 0; i < (n - 1) - 1; i++)
	{
		if (ds[i] != 0)
		{
			// 曲率 = 航向角变化率 / 弧长增量
			c[i] = (yaw[i + 1] - yaw[i]) / ds[i];
		}
		else
		{
			// 避免除零错误
			c[i] = FLT_MAX;
		}
	}
}

/**
 * @brief 检查轨迹是否满足约束条件和碰撞检测
 * 
 * 该函数检查生成的轨迹是否满足以下条件：
 * 1. 初始航向角与机器人当前航向角的偏差在允许范围内
 * 2. 轨迹不与障碍物发生碰撞
 * 
 * @param fp 待检查的Frenet轨迹
 * @param bot_yaw 机器人当前航向角
 * @param yaw_error 允许的航向角偏差
 * @param obst_r 障碍物检测半径
 * @return true 轨迹满足所有约束条件
 * @return false 轨迹违反某项约束条件
 */
bool check_path(FrenetPath fp, double bot_yaw, double yaw_error,
				double obst_r)
{
	int flag = 0;
	vecD path_yaw = fp.get_yaw();
	if (path_yaw.size() == 0)
		return 0;
	// 检查初始航向角偏差是否超过允许范围（默认20度）
	if ((path_yaw[0] - bot_yaw) > yaw_error || (path_yaw[0] - bot_yaw) < -yaw_error)
	{
		flag = 1;
	}
	if (flag == 1)
	{
		return 0;
	}
	else if (fp.check_collision(obst_r) == 0)
	{
		return 1;
	}
	return 0;
}

/**
 * @brief 变换机器人足迹到指定位置和姿态
 * 
 * 该函数将机器人的足迹从当前位置变换到目标位置，
 * 用于碰撞检测中计算机器人在轨迹各点的实际占用空间。
 * 
 * 变换步骤：
 * 1. 从四元数中提取当前姿态角
 * 2. 计算目标姿态与当前姿态的角度差
 * 3. 对足迹的每个点进行旋转和平移变换
 * 
 * @param fp 机器人足迹点集
 * @param cp 机器人当前位姿
 * @param px 目标位置x坐标
 * @param py 目标位置y坐标
 * @param pyaw 目标姿态角
 * @return 变换后的足迹点集
 */
vector<geometry_msgs::Point32> transformation(vector<geometry_msgs::Point32> fp,
											  geometry_msgs::Pose cp, double px, double py, double pyaw)
{
	vector<geometry_msgs::Point32> new_fp(fp.size());
	// 从四元数中提取欧拉角
	tf::Quaternion qb(cp.orientation.x, cp.orientation.y, cp.orientation.z, cp.orientation.w);
	tf::Matrix3x3 mb(qb);
	double broll, bpitch, byaw;
	mb.getRPY(broll, bpitch, byaw);
	// 获取当前位置
	double bx, by;
	bx = cp.position.x;
	by = cp.position.y;
	// 计算变换参数
	double x, y, theta;
	theta = pyaw - byaw;  // 角度差
	x = px - bx;         // x方向位移
	y = py - by;         // y方向位移
	// 对每个足迹点进行变换
	int n = new_fp.size();
	for (int i = 0; i < n; i++)
	{
		// 应用旋转和平移变换
		new_fp[i].x = (fp[i].x - bx) * cos(theta) + (fp[i].y - by) * sin(theta) + x + bx;
		new_fp[i].y = -(fp[i].x - bx) * sin(theta) + (fp[i].y - by) * cos(theta) + y + by;
	}
	return new_fp;
}

/**
 * @brief 计算两点之间的欧几里得距离
 * 
 * 使用SIMD指令优化的距离计算函数，用于高效的碰撞检测。
 * 
 * @param x1 第一个点的x坐标
 * @param y1 第一个点的y坐标
 * @param x2 第二个点的x坐标
 * @param y2 第二个点的y坐标
 * @return 两点之间的距离
 */
#pragma omp declare simd
double dist(double x1, double y1, double x2, double y2)
{
	return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

/**
 * @brief 检查单个点是否与障碍物发生碰撞
 * 
 * 该函数使用优化的碰撞检测算法，通过在x和y轴上查找最近的障碍物边界
 * 来快速判断点是否与障碍物发生碰撞。使用二分查找提高检索效率。
 * 
 * 算法步骤：
 * 1. 在x轴排序的障碍物列表中找到最近的上下边界
 * 2. 计算点到这些边界障碍物的距离
 * 3. 在y轴排序的障碍物列表中重复相同过程
 * 4. 如果任一方向的最小距离小于安全半径，则判定为碰撞
 * 
 * @param p 待检查的点（几何消息格式）
 * @param obst_r 障碍物安全检测半径
 * @return 1 该点与障碍物发生碰撞
 * @return 0 该点安全，未发生碰撞
 */
bool point_obcheck(geometry_msgs::Point32 p, double obst_r)
{
	int xlower, ylower, xupper, yupper;
	// 在x轴排序的障碍物列表中进行二分查找
	auto it = lower_bound(ob_x.begin(), ob_x.end(), p.x);
	if (ob_x.size() == 0)
	{
		return 0; // 无障碍物，安全
	}
	if (it == ob_x.begin())
	{
		xlower = xupper = it - ob_x.begin(); // 没有更小的值
	}
	else if (it == ob_x.end())
	{
		xupper = xlower = (it - 1) - ob_x.begin(); // 没有更大的值
	}
	else
	{
		xlower = (it - 1) - ob_x.begin(); // 下边界索引
		xupper = it - ob_x.begin();       // 上边界索引
	}
	// 计算到x轴方向最近障碍物的距离
	double dist1 = dist(p.x, p.y, ob_x[xlower], ob_y[xlower]);
	double dist2 = dist(p.x, p.y, ob_x[xupper], ob_y[yupper]);
	if (min(dist1, dist2) < obst_r)
	{
		return 1; // 碰撞检测到
	}
	// 在y轴排序的障碍物列表中进行二分查找
	it = lower_bound(ob_y.begin(), ob_y.end(), p.y);
	if (it == ob_y.begin())
	{
		ylower = yupper = it - ob_y.begin(); // 没有更小的值
	}
	else if (it == ob_y.end())
	{
		yupper = ylower = (it - 1) - ob_y.begin(); // 没有更大的值
	}
	else
	{
		ylower = (it - 1) - ob_y.begin(); // 下边界索引
		yupper = it - ob_y.begin();       // 上边界索引
	}
	// 计算到y轴方向最近障碍物的距离
	dist1 = dist(p.x, p.y, ob_x[ylower], ob_y[ylower]);
	dist2 = dist(p.x, p.y, ob_x[yupper], ob_y[yupper]);
	if (min(dist1, dist2) < obst_r)
	{
		return 1; // 碰撞检测到
	}
	return 0; // 安全，无碰撞
}

/**
 * @brief 检查机器人轨迹是否与障碍物发生碰撞
 * 
 * 该函数检查机器人沿着整条轨迹运动时是否会与障碍物发生碰撞。
 * 对轨迹上的每个点，都会将机器人足迹变换到该位置进行碰撞检测。
 * 
 * 检测步骤：
 * 1. 验证轨迹数据的完整性
 * 2. 遍历轨迹上的每个位置点
 * 3. 将机器人足迹变换到当前位置和姿态
 * 4. 检查变换后的足迹是否与障碍物碰撞
 * 
 * @param obst_r 障碍物安全检测半径
 * @return 1 轨迹与障碍物发生碰撞
 * @return 0 轨迹安全，未发生碰撞
 */
bool FrenetPath::check_collision(double obst_r)
{
	// 检查轨迹数据完整性
	if (s.size() != x.size())
	{
		return 1; // 数据不完整，认为不安全
	}
	// 遍历轨迹上的每个点进行碰撞检测
	for (unsigned int i = 0; i < min(x.size(), yaw.size()); i++)
	{
		// 将机器人足迹变换到轨迹点(x[i], y[i], yaw[i])
		vector<geometry_msgs::Point32> trans_footprint = transformation(footprint.polygon.points,
																		odom.pose.pose, x[i], y[i], yaw[i]);
		// 检查变换后足迹的每个点是否与障碍物碰撞
		for (unsigned int j = 0; j < trans_footprint.size(); j++)
		{
			if (point_obcheck(trans_footprint[j], obst_r) == 1)
			{
				return 1; // 检测到碰撞
			}
		}
	}
	return 0; // 轨迹安全
}

/**
 * @brief 轨迹成本比较函数
 * 
 * 用于对FrenetPath对象按照成本进行排序的比较函数。
 * 首先按总成本(cf)排序，如果总成本相同则按急动度成本排序。
 * 
 * 成本计算包括：
 * - 总成本(cf)：综合考虑安全性、舒适性、效率的总体成本
 * - 急动度成本：横向急动度(Jp)和纵向急动度(Js)的加权和
 * 
 * @param a 第一个轨迹
 * @param b 第二个轨迹
 * @return true 如果轨迹a的成本低于轨迹b
 */
inline bool sortByCost(FrenetPath &a, FrenetPath &b)
{
	// 首先按总成本排序
	if (a.get_cf() != b.get_cf())
	{
		return a.get_cf() < b.get_cf();
	}
	else
	{
		// 总成本相同时，按急动度成本排序
		double jerkCost1, jerkCost2;
		jerkCost1 = KLAT * a.get_Jp() + KLON * a.get_Js(); // 轨迹a的急动度成本
		jerkCost2 = KLAT * b.get_Jp() + KLON * b.get_Js(); // 轨迹b的急动度成本
		return jerkCost1 < jerkCost2;
	}
}

/**
 * @brief 绘制轨迹路径
 * 
 * 使用matplotlib-cpp库在二维平面上绘制轨迹的x-y坐标路径。
 * 轨迹以红色实线显示，用于可视化和调试。
 */
// 可视化函数已移除，不再需要matplotlib-cpp
// plot_path() 和 plot_velocity_profile() 函数实现已删除

// 移除了未使用的可视化相关变量

/**
 * @brief 显示多条轨迹路径
 * 
 * 批量显示候选轨迹集合中的所有轨迹路径，
 * 用于可视化轨迹采样结果和算法的搜索空间。
 * 
 * @param fplist 候选轨迹列表
 */
void display_paths(vector<FrenetPath> fplist)
{
	// 可视化功能已移除，不再需要matplotlib
	// 该函数保留为空实现以保持接口兼容性
}

/**
 * @brief Frenet坐标系最优轨迹规划主函数
 * 
 * 该函数是整个Frenet轨迹规划算法的核心，实现了完整的轨迹生成、
 * 评估和选择流程。算法步骤如下：
 * 
 * 1. 轨迹采样：基于当前状态生成候选轨迹集合
 * 2. 坐标转换：将Frenet轨迹转换为全局坐标轨迹
 * 3. 约束检查：验证轨迹是否满足运动学和动力学约束
 * 4. 碰撞检测：检查轨迹是否与障碍物发生碰撞
 * 5. 成本计算：为有效轨迹计算综合成本函数
 * 6. 最优选择：选择成本最低的轨迹作为最优解
 * 
 * @param csp 参考路径的三次样条插值器
 * @param s0 当前纵向位置（沿参考路径的弧长）
 * @param c_speed 当前纵向速度
 * @param c_d 当前横向位移
 * @param c_d_d 当前横向速度
 * @param c_d_dd 当前横向加速度
 * @param lp 上一次规划的轨迹路径
 * @param bot_yaw 机器人当前航向角
 * @return 最优的Frenet轨迹
 */
FrenetPath frenet_optimal_planning(Spline2D csp, double s0, double c_speed, double c_d,
								   double c_d_d, double c_d_dd, FrenetPath lp, double bot_yaw)
{
	// 使用静态变量保存最优路径，确保在未找到新路径时车辆能继续沿之前路径行驶
	// 除非设置了STOP_CAR标志
	static FrenetPath bestpath;
	if (STOP_CAR)
	{
		FrenetPath empty;
		bestpath = empty;
	}

	// 步骤1：初始化Fplist类对象，生成候选轨迹集合
	// fplist_lat成员变量存储采样得到的Frenet轨迹
	static Fplist fplist_obj(c_speed, c_d, c_d_d, c_d_dd, s0);
	fplist_obj = Fplist(c_speed, c_d, c_d_d, c_d_dd, s0);
	current_fplist = &fplist_obj;  // 设置全局指针以供publishSampledPaths使用
	vector<FrenetPath> fplist = fplist_obj.fplist_lat;

	// 步骤2-6：按成本排序候选轨迹，依次进行碰撞检测直到找到可行轨迹
	std::sort(fplist.begin(), fplist.end(), sortByCost);
	for (int i = 0; i < fplist.size(); i++)
	{
		// 将Frenet轨迹转换为全局坐标系轨迹
		fplist[i] = calc_global_path(fplist[i], csp);
		// 检查轨迹约束和碰撞（航向角偏差限制0.523599弧度≈30度，障碍物半径2.0米）
		if (check_path(fplist[i], bot_yaw, 0.523599, 2.0) == 1)
		{
			// 找到第一个满足所有约束的轨迹，设为最优轨迹
			bestpath = fplist[i];
			break;
		}
	}

	return bestpath;
}

/**
 * @brief Fplist类构造函数
 * 
 * 该构造函数实现完整的Frenet轨迹采样和成本计算流程：
 * 1. 生成横向轨迹采样集合
 * 2. 生成纵向轨迹采样集合
 * 3. 将横向和纵向轨迹进行组合
 * 4. 计算每条轨迹的综合成本
 * 
 * @param c_speedc 当前纵向速度
 * @param c_dc 当前横向位移
 * @param c_d_dc 当前横向速度
 * @param c_d_ddc 当前横向加速度
 * @param s00 当前纵向位置
 */
Fplist::Fplist(double c_speedc, double c_dc, double c_d_dc, double c_d_ddc, double s00)
{
	c_speed = c_speedc;
	c_d = c_dc;
	c_d_d = c_d_dc;
	c_d_dd = c_d_ddc;
	s0 = s00;

	// 步骤1：生成横向轨迹采样集合
	// 根据是否停车采用不同的采样策略
	if (STOP_CAR)
	{
		// 停车模式：目标横向速度固定为0
		for (double di = -MAX_ROAD_WIDTH; di < MAX_ROAD_WIDTH; di += D_ROAD_W)
		{
			for (double Ti = MINT; Ti < MAXT; Ti += DT)
			{
				FrenetPath temp = calc_lat(di, Ti, 0.0);
				for (int p = 0; p < samples_tv; p++)
				{
					fplist_lat.push_back(temp);
				}
			}
		}
	}
	else
	{
		// 正常行驶模式：在横向速度范围内采样
		for (double di = -MAX_ROAD_WIDTH; di < MAX_ROAD_WIDTH; di += D_ROAD_W)
		{
			for (double Ti = MINT; Ti < MAXT; Ti += DT)
			{
				for (double Di_d = -MAX_LAT_VEL; Di_d < MAX_LAT_VEL + 0.001; Di_d += D_D_NS)
				{
					FrenetPath temp = calc_lat(di, Ti, Di_d);
					for (int p = 0; p < samples_tv; p++)
					{
						fplist_lat.push_back(temp);
					}
				}
			}
		}
	}

	// 步骤2：生成纵向轨迹采样集合
	if (STOP_CAR)
	{
		// 停车模式：目标速度固定为TARGET_SPEED
		for (double Ti = MINT; Ti < MAXT; Ti += DT)
		{
			fplist_lon.push_back(calc_lon(TARGET_SPEED, Ti));
		}
	}
	else
	{
		// 正常行驶模式：在目标速度范围内采样
		for (double Ti = MINT; Ti < MAXT; Ti += DT)
		{
			for (double tv = TARGET_SPEED - D_T_S * N_S_SAMPLE; tv < TARGET_SPEED + D_T_S * N_S_SAMPLE; tv += D_T_S)
			{
				fplist_lon.push_back(calc_lon(tv, Ti));
			}
		}
	}

	// 步骤3：将纵向轨迹信息复制到对应的横向轨迹中
	for (int i = 0; i < fplist_lat.size(); i += samples_tv)
	{
		copy(i);
	}

	// 步骤4：计算每条轨迹的综合成本
	for (int i = 0; i < fplist_lat.size(); i++)
	{
		calc_cost(i);
	}
}

/**
 * @brief 计算横向（侧向）轨迹部分
 * 
 * 使用五次多项式生成从当前横向状态到目标横向状态的轨迹。
 * 横向轨迹描述车辆在Frenet坐标系中垂直于参考路径的运动。
 * 
 * @param di 目标横向位移（相对于参考路径的距离）
 * @param Ti 轨迹持续时间
 * @param Di_d 目标横向速度
 * @return FrenetPath 包含横向轨迹信息的路径对象
 */
FrenetPath Fplist::calc_lat(double di, double Ti, double Di_d)
{
	FrenetPath fp;

	// 根据时间步长计算轨迹点数量
	int n = 1 + Ti / DT;
	fp.t.resize(n);      // 时间序列
	fp.d.resize(n);      // 横向位移序列
	fp.d_d.resize(n);    // 横向速度序列
	fp.d_dd.resize(n);   // 横向加速度序列
	fp.d_ddd.resize(n);  // 横向加加速度序列

	// 构造五次多项式：从当前状态(c_d, c_d_d, c_d_dd)到目标状态(di, Di_d, 0.0)
	// 边界条件：起始横向位移、速度、加速度 -> 目标横向位移、速度、加速度(0)
	QuinticPolynomial lat_qp(c_d, c_d_d, c_d_dd, di, Di_d, 0.0, Ti);

	// 按时间步长采样轨迹点
	for (int te = 0; te < n; te++)
	{
		double t = te * DT;
		fp.t[te] = t;                                    // 时间点
		fp.d[te] = lat_qp.calc_point(t);                 // 横向位移
		fp.d_d[te] = lat_qp.calc_first_derivative(t);    // 横向速度
		fp.d_dd[te] = lat_qp.calc_second_derivative(t);  // 横向加速度
		fp.d_ddd[te] = lat_qp.calc_third_derivative(t);  // 横向加加速度
	}

	// 计算横向轨迹的成本函数
	vecD d_ddd_vec = fp.d_ddd;
	// Jp：加加速度的平方积分，衡量轨迹平滑度
	fp.Jp = inner_product(d_ddd_vec.begin(), d_ddd_vec.end(), d_ddd_vec.begin(), 0);
	fp.Ti = Ti;  // 轨迹时间
	// cd：横向成本 = 平滑度成本 + 时间成本 + 偏离参考路径成本
	fp.cd = KJ * fp.Jp + KT * Ti + KD * std::pow(fp.d.back(), 2);
	return fp;
}

/**
 * @brief 计算纵向轨迹部分
 * 
 * 使用五次多项式生成从当前纵向状态到目标纵向状态的轨迹。
 * 纵向轨迹描述车辆在Frenet坐标系中沿着参考路径的运动。
 * 
 * @param tv 目标纵向速度
 * @param Ti 轨迹持续时间
 * @return FrenetPath 包含纵向轨迹信息的路径对象
 */
FrenetPath Fplist::calc_lon(double tv, double Ti)
{
	FrenetPath fp;
	// 构造五次多项式：从当前状态(s0, c_speed, 0.0)到目标状态
	// 目标纵向位移限制在当前位置+15米和目的地之间的较小值
	// 边界条件：起始位置、速度、加速度(0) -> 目标位置、速度、加速度(0)
	QuinticPolynomial lon_qp(s0, c_speed, 0.0, min(s0 + 15, s_dest), tv, 0.0, Ti);

	// 根据时间步长计算轨迹点数量
	int n = 1 + Ti / DT;
	fp.t.resize(n);      // 时间序列
	fp.s.resize(n);      // 纵向位移序列
	fp.s_d.resize(n);    // 纵向速度序列
	fp.s_dd.resize(n);   // 纵向加速度序列
	fp.s_ddd.resize(n);  // 纵向加加速度序列

	// 按时间步长采样轨迹点
	for (int te = 0; te < n; te++)
	{
		double t = te * DT;
		fp.t[te] = t;                                    // 时间点
		fp.s[te] = lon_qp.calc_point(t);                 // 纵向位移
		fp.s_d[te] = lon_qp.calc_first_derivative(t);    // 纵向速度
		fp.s_dd[te] = lon_qp.calc_second_derivative(t);  // 纵向加速度
		fp.s_ddd[te] = lon_qp.calc_third_derivative(t);  // 纵向加加速度
	}
	
	// 计算纵向轨迹的成本函数
	fp.Ti = Ti;  // 轨迹时间
	vecD s_ddd_vec = fp.s_ddd;
	// Js：加加速度的平方积分，衡量轨迹平滑度
	fp.Js = inner_product(s_ddd_vec.begin(), s_ddd_vec.end(), s_ddd_vec.begin(), 0);
	// dss：终端速度与目标速度的偏差平方
	fp.dss = std::pow(TARGET_SPEED - fp.s_d.back(), 2);
	// cv：纵向成本 = 平滑度成本 + 时间成本 + 速度偏差成本
	fp.cv = KJ * fp.Js + KT * fp.Ti + KD * fp.dss;
	return fp;
}

/**
 * @brief 将纵向轨迹信息复制到对应的横向轨迹中
 * 
 * 该函数实现横向和纵向轨迹的组合，将预先计算好的纵向轨迹信息
 * 复制到对应时间的横向轨迹中，形成完整的Frenet轨迹。
 * 
 * @param i 横向轨迹列表中的起始索引
 */
void Fplist::copy(int i)
{
	// 根据轨迹时间计算对应纵向轨迹的起始索引
	// 纵向轨迹按时间分组存储，每个时间组有samples_tv个速度采样
	int index_start = ((fplist_lat[i]).Ti - MINT) * samples_tv;

	// 将纵向轨迹信息复制到对应的横向轨迹组中
	for (int j = 0; j < samples_tv; j++)
	{
		// 复制纵向位移、速度、加速度、加加速度信息
		fplist_lat[i + j].s = fplist_lon[index_start + j].s;       // 纵向位移序列
		fplist_lat[i + j].s_d = fplist_lon[index_start + j].s_d;   // 纵向速度序列
		fplist_lat[i + j].s_dd = fplist_lon[index_start + j].s_dd; // 纵向加速度序列
		fplist_lat[i + j].s_ddd = fplist_lon[index_start + j].s_ddd; // 纵向加加速度序列
		
		// 复制纵向成本相关信息
		fplist_lat[i + j].Js = fplist_lon[index_start + j].Js;     // 纵向平滑度成本
		fplist_lat[i + j].dss = fplist_lon[index_start + j].dss;   // 速度偏差成本
		fplist_lat[i + j].cv = fplist_lon[index_start + j].cv;     // 纵向总成本
	}
}

/**
 * @brief 计算轨迹的最终综合成本
 * 
 * 将横向成本和纵向成本按权重组合，得到轨迹的最终评价指标。
 * 成本越低的轨迹越优。
 * 
 * @param i 轨迹在fplist_lat中的索引
 */
void Fplist::calc_cost(int i)
{
	// cf：最终成本 = 横向成本权重 × 横向成本 + 纵向成本权重 × 纵向成本
	fplist_lat[i].cf = KLAT * fplist_lat[i].cd + KLON * fplist_lat[i].cv;
}
