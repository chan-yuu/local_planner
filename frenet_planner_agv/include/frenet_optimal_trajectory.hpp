/**
 * @file frenet_optimal_trajectory.hpp
 * @brief Frenet坐标系最优轨迹规划器头文件
 * @details 实现基于Frenet坐标系的最优轨迹规划算法，包括轨迹生成、碰撞检测、成本评估等功能
 * @author CYUN
 * @date 2025
 * 
 * 主要功能：
 * - 在Frenet坐标系下生成候选轨迹
 * - 进行碰撞检测和约束检查
 * - 基于多目标成本函数选择最优轨迹
 * - 将Frenet轨迹转换为笛卡尔坐标系轨迹
 */

#ifndef FRENET_OPTIMAL_TRAJECTORY_HPP_
#define FRENET_OPTIMAL_TRAJECTORY_HPP_

#include <algorithm>
#include <cfloat>
#include "../include/polynomials.hpp"
#include "../include/cubic_spline_planner.hpp"
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PolygonStamped.h>
#include <vector>
//#include "tbb/concurrent_vector.h"

// Parameters
double MAX_SPEED;	   // maximum speed [m/s]
double MAX_ACCEL;	   // maximum acceleration [m/ss]
double MAX_CURVATURE;  // maximum curvature [1/m]
double MAX_ROAD_WIDTH; // maximum road width [m]
double D_ROAD_W;	   // road width sampling length [m]
double DT;			   // time tick [s]
double MAXT;		   // max prediction time [m]
double MINT;		   // min prediction time [m]
double TARGET_SPEED;   // target speed [m/s]
double D_T_S;		   // target speed sampling length [m/s]
double N_S_SAMPLE;	   // sampling number of target speed
double ROBOT_RADIUS;   // robot radius [m]
double MIN_LAT_VEL;	   // minimum lateral speed. Sampling for d_dot
double MAX_LAT_VEL;	   // maxmum lateral speed. Sampling for d_dot
double D_D_NS;		   // Step size for sampling of d_dot
double MAX_SHIFT_D;	   // Sampling width for sampling of d.
double RATE;		   // Loop execution rate [Hz]

// cost weights
double KJ;
double KT;
double KD;
double KD_V;
double KLAT;
double KLON;
bool STOP_CAR = false;
double s_dest;
// Waypoints
vector<double> W_X;
vector<double> W_Y;

nav_msgs::Odometry odom;
nav_msgs::OccupancyGrid cmap;
geometry_msgs::PolygonStamped footprint;
vector<double> ob_x; // x coordinates of the obstacles
vector<double> ob_y; // y coordinates of the obstacles

/**
 * @brief Frenet路径类
 * @details 存储完整的Frenet轨迹信息，包括Frenet坐标和笛卡尔坐标表示
 */
class FrenetPath
{
private:
	// 时间序列和Frenet坐标系轨迹数据
	vecD t;        ///< 时间点序列
	vecD d;        ///< 横向位移序列
	vecD d_d;      ///< 横向速度序列
	vecD d_dd;     ///< 横向加速度序列
	vecD d_ddd;    ///< 横向急动度序列
	vecD s;        ///< 纵向位移序列
	vecD s_d;      ///< 纵向速度序列
	vecD s_dd;     ///< 纵向加速度序列
	vecD s_ddd;    ///< 纵向急动度序列
	
	// 笛卡尔坐标系轨迹数据
	vecD x;        ///< x坐标序列
	vecD y;        ///< y坐标序列
	vecD yaw;      ///< 航向角序列
	vecD ds;       ///< 弧长增量序列
	vecD c;        ///< 曲率序列
	
	// 成本函数和约束参数
	double Js;     ///< 横向急动度成本（d_ddd的平方和）
	double Jp;     ///< 纵向急动度成本（s_ddd的平方和）
	double cd;     ///< 横向轨迹成本
	double cv;     ///< 纵向轨迹成本
	double cf;     ///< 总成本（cd和cv的加权和）
	double Ti;     ///< 轨迹时间长度
	double dss;    ///< 弧长步长
public:
    // ==================== Getter函数 ====================
    vecD get_t();           ///< 获取时间序列
    vecD get_d();           ///< 获取横向位移序列
    vecD get_d_d();         ///< 获取横向速度序列
    vecD get_d_dd();        ///< 获取横向加速度序列
    vecD get_d_ddd();       ///< 获取横向急动度序列
    vecD get_s();           ///< 获取纵向位移序列
    vecD get_s_d();         ///< 获取纵向速度序列
    vecD get_s_dd();        ///< 获取纵向加速度序列
    vecD get_s_ddd();       ///< 获取纵向急动度序列
    vecD get_x();           ///< 获取x坐标序列
    vecD get_y();           ///< 获取y坐标序列
    vecD get_yaw();         ///< 获取航向角序列
    vecD get_ds();          ///< 获取弧长增量序列
    vecD get_c();           ///< 获取曲率序列
    double get_Js();        ///< 获取横向急动度成本
    double get_Jp();        ///< 获取纵向急动度成本
    double get_cd();        ///< 获取横向轨迹成本
    double get_cv();        ///< 获取纵向轨迹成本
    double get_cf();        ///< 获取总成本
    double get_Ti();        ///< 获取轨迹时间长度
    double get_dss();       ///< 获取弧长步长
    
    // ==================== Setter函数 ====================
    void set_t(vecD t_);           ///< 设置时间序列
    void set_d(vecD d_);           ///< 设置横向位移序列
    void set_d_d(vecD d_d_);       ///< 设置横向速度序列
    void set_d_dd(vecD d_dd_);     ///< 设置横向加速度序列
    void set_d_ddd(vecD d_ddd_);   ///< 设置横向急动度序列
    void set_s(vecD s_);           ///< 设置纵向位移序列
    void set_s_d(vecD s_d_);       ///< 设置纵向速度序列
    void set_s_dd(vecD s_dd_);     ///< 设置纵向加速度序列
    void set_s_ddd(vecD s_ddd_);   ///< 设置纵向急动度序列
    void set_x(vecD x_);           ///< 设置x坐标序列
    void set_y(vecD y_);           ///< 设置y坐标序列
    void set_yaw(vecD yaw_);       ///< 设置航向角序列
    void set_ds(vecD ds_);         ///< 设置弧长增量序列
    void set_c(vecD c_);           ///< 设置曲率序列
    void set_Js(double Js_);       ///< 设置横向急动度成本
    void set_Jp(double Jp_);       ///< 设置纵向急动度成本
    void set_cd(double cd_);       ///< 设置横向轨迹成本
    void set_cv(double cv_);       ///< 设置纵向轨迹成本
    void set_cf(double cf_);       ///< 设置总成本
    void set_Ti(double Ti_);       ///< 设置轨迹时间长度
    void set_dss(double dss_);     ///< 设置弧长步长

    // ==================== 核心算法函数 ====================
    
    /**
     * @brief 计算横向（d方向）轨迹
     * @param c_d 当前横向位移
     * @param c_d_d 当前横向速度
     * @param c_d_dd 当前横向加速度
     * @param s 目标横向位移
     * @param Ti 轨迹时间长度
     * @param dt 时间步长
     */
    void calc_lat_paths(double c_d, double c_d_d, double c_d_dd, double s, double Ti, double dt);
    
    /**
     * @brief 计算纵向（s方向）轨迹
     * @param c_s 当前纵向位移
     * @param c_s_d 当前纵向速度
     * @param c_s_dd 当前纵向加速度
     * @param target_speed 目标速度
     * @param Ti 轨迹时间长度
     */
    void calc_lon_paths(double c_s, double c_s_d, double c_s_dd, double target_speed, double Ti);
    
    /**
     * @brief 使用五次多项式计算纵向轨迹
     * @param c_s 当前纵向位移
     * @param c_s_d 当前纵向速度
     * @param c_s_dd 当前纵向加速度
     * @param target_s 目标纵向位移
     * @param Ti 轨迹时间长度
     */
    void calc_lon_paths_quintic_poly(double c_s, double c_s_d, double c_s_dd, double target_s, double Ti);
    
    /**
     * @brief 将Frenet坐标轨迹转换为全局笛卡尔坐标轨迹
     * @param csp 参考路径的三次样条插值器
     */
    void adding_global_path(Spline2D csp);
    
    /**
     * @brief 将Frenet坐标轨迹转换为全局笛卡尔坐标轨迹（改进版本）
     * @param csp 参考路径的三次样条插值器（引用传递）
     */
    void calc_global_path(Spline2D &csp);
    
    /**
     * @brief 检查轨迹是否与障碍物发生碰撞
     * @param robot_radius 机器人半径
     * @return true表示发生碰撞，false表示无碰撞
     */
    bool check_collision(double robot_radius);
    
    // Removed plot_path() and plot_velocity_profile() functions (visualization no longer needed)
    // ==================== 友元函数和操作符重载 ====================
    
    /**
     * @brief 输出流操作符重载，用于打印FrenetPath对象
     */
	friend ostream &operator<<(ostream &os, const FrenetPath &fp);
	
	/**
	 * @brief 友元类声明，允许Fplist类访问私有成员
	 */
	friend class Fplist;
	
	/**
	 * @brief 大于操作符重载，基于总成本cf进行比较
	 * @param other 另一个FrenetPath对象
	 * @return true表示当前对象成本更高
	 */
	bool operator>(FrenetPath &other)
	{
		return (cf > other.get_cf());
	}
	
	/**
	 * @brief 小于操作符重载，基于总成本cf进行比较
	 * @param other 另一个FrenetPath对象
	 * @return true表示当前对象成本更低
	 */
	bool operator<(FrenetPath &other)
	{
		return (cf < other.get_cf());
	}
};

// ==================== 全局函数声明 ====================

/**
 * @brief 计算两点之间的欧几里得距离
 * @param x1 第一个点的x坐标
 * @param y1 第一个点的y坐标
 * @param x2 第二个点的x坐标
 * @param y2 第二个点的y坐标
 * @return 两点之间的距离
 */
double dist(double x1, double y1, double x2, double y2);

/**
 * @brief 获取轨迹的横向位移范围
 * @param fp FrenetPath轨迹对象
 * @param min_d 输出参数：最小横向位移
 * @param max_d 输出参数：最大横向位移
 */
void get_limits_d(FrenetPath fp, double *min_d, double *max_d);

/**
 * @brief 检查轨迹路径的有效性（速度、加速度、急动度约束）
 * @param fplist 轨迹路径列表
 * @param max_speed 最大速度限制
 * @param max_accel 最大加速度限制
 * @param max_curvature 最大曲率限制
 * @return 通过检查的有效轨迹列表
 */
vector<FrenetPath> check_path(vector<FrenetPath> &fplist, double max_speed, double max_accel, double max_curvature);

/**
 * @brief 生成Frenet坐标系下的候选轨迹路径
 * @param c_speed 当前速度
 * @param c_d 当前横向位移
 * @param c_d_d 当前横向速度
 * @param c_d_dd 当前横向加速度
 * @param s0 当前纵向位移
 * @param fp 参考轨迹路径
 * @return 生成的候选轨迹路径列表
 */
vector<FrenetPath> calc_frenet_paths(double c_speed, double c_d, double c_d_d, double c_d_dd, double s0, FrenetPath fp);

/**
 * @brief 将Frenet坐标轨迹转换为全局笛卡尔坐标轨迹
 * @param fplist Frenet坐标轨迹列表
 * @param csp_ds 参考路径的弧长步长
 * @return 转换后的全局坐标轨迹列表
 */
vector<FrenetPath> calc_global_paths(vector<FrenetPath> &fplist, double csp_ds);

/**
 * @brief Frenet最优轨迹规划主函数
 * @param csp 参考路径的三次样条插值器
 * @param s0 当前纵向位移
 * @param c_speed 当前速度
 * @param c_d 当前横向位移
 * @param c_d_d 当前横向速度
 * @param c_d_dd 当前横向加速度
 * @param fp 参考轨迹路径
 * @param robot_radius 机器人半径
 * @return 最优轨迹路径
 */
FrenetPath frenet_optimal_planning(Spline2D csp, double s0, double c_speed, double c_d, double c_d_d, double c_d_dd, FrenetPath fp, double robot_radius);

/*******for printing the frenet path***********/
template <class T>
ostream &operator<<(ostream &os, vector<T> V)
{
	os << "[ ";
	for (auto v : V)
		os << v << " ";
	return os << "]";
}

#define trace(...) ((void)0)
ostream &operator<<(ostream &os, const FrenetPath &fp)
{
	(void)fp; // Suppress unused parameter warning
	trace(fp.t);
	trace(fp.d);
	trace(fp.d_d);
	trace(fp.d_dd);
	trace(fp.d_ddd);
	trace(fp.s);
	trace(fp.s_d);
	trace(fp.s_dd);
	trace(fp.s_ddd);
	trace(fp.x);
	trace(fp.y);
	trace(fp.yaw);
	trace(fp.ds);
	return os;
}
/***********************end******************/
inline vecD FrenetPath::get_t()
{
	return t;
}
inline vecD FrenetPath::get_d()
{
	return d;
}
inline vecD FrenetPath::get_d_d()
{
	return d_d;
}
inline vecD FrenetPath::get_d_dd()
{
	return d_dd;
}
inline vecD FrenetPath::get_d_ddd()
{
	return d_ddd;
}
inline vecD FrenetPath::get_s()
{
	return s;
}
inline vecD FrenetPath::get_s_d()
{
	return s_d;
}
inline vecD FrenetPath::get_s_dd()
{
	return s_dd;
}
inline vecD FrenetPath::get_s_ddd()
{
	return s_ddd;
}
inline vecD FrenetPath::get_x()
{
	return x;
}
inline vecD FrenetPath::get_y()
{
	return y;
}
inline vecD FrenetPath::get_yaw()
{
	return yaw;
}
inline vecD FrenetPath::get_ds()
{
	return ds;
}
inline vecD FrenetPath::get_c()
{
	return c;
}
inline double FrenetPath::get_cf()
{
	return cf;
}
inline void FrenetPath::set_Jp(double jp)
{
	Jp = jp;
}
inline void FrenetPath::set_Js(double js)
{
	Js = js;
}
inline double FrenetPath::get_Jp()
{
	return Jp;
}
inline double FrenetPath::get_Js()
{
	return Js;
}

/**
 * @brief Frenet路径列表管理类
 * 
 * 该类用于管理和生成Frenet坐标系下的轨迹路径列表，
 * 包括横向轨迹和纵向轨迹的计算和管理。
 */
class Fplist
{
public:
    /**
     * @brief 构造函数
     * @param c_speed 当前速度
     * @param c_d 当前横向位移
     * @param c_d_d 当前横向速度
     * @param c_d_dd 当前横向加速度
     * @param s0 当前纵向位移
     */
	Fplist(double c_speed, double c_d, double c_d_d, double c_d_dd, double s0);
	
	/**
	 * @brief 计算横向轨迹
	 * @param di 目标横向位移
	 * @param Ti 轨迹时间长度
	 * @param Di_d 目标横向速度
	 * @return 计算得到的横向轨迹
	 */
	FrenetPath calc_lat(double di, double Ti, double Di_d);
	
	/**
	 * @brief 计算纵向轨迹
	 * @param tv 目标速度
	 * @param Ti 轨迹时间长度
	 * @return 计算得到的纵向轨迹
	 */
	FrenetPath calc_lon(double tv, double Ti);
	
	/**
	 * @brief 复制轨迹数据
	 * @param i 轨迹索引
	 */
	void copy(int i);
	
	/**
	 * @brief 计算轨迹成本
	 * @param i 轨迹索引
	 */
	void calc_cost(int i);

    // ==================== 成员变量 ====================
    
	double c_speed;     ///< 当前速度
	double c_d;         ///< 当前横向位移
	double c_d_d;       ///< 当前横向速度
	double c_d_dd;      ///< 当前横向加速度
	double s0;          ///< 当前纵向位移
	
	/// 横向轨迹路径列表
	std::vector<FrenetPath> fplist_lat;
	
	/// 纵向轨迹路径列表
	std::vector<FrenetPath> fplist_lon;
	
	/// 目标速度采样数量（默认为2倍的纵向采样数）
	int samples_tv = 2 * N_S_SAMPLE;
};

/**
 * @brief 获取当前Fplist对象指针
 * @return 当前Fplist对象的指针，用于访问所有采样路径
 */
Fplist* get_current_fplist();

#endif // FRENET_OPTIMAL_TRAJECTORY_HPP_
