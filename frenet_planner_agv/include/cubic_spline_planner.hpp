/**
 * @file cubic_spline_planner.hpp
 * @brief 三次样条插值路径规划器头文件
 * @details 实现了一维和二维三次样条插值算法，用于生成平滑的路径曲线
 *          支持位置、速度、加速度和曲率的计算
 * @author CYUN
 * @date 2025
 */

#ifndef CUBIC_SPLINE_PLANNER_HPP_
#define CUBIC_SPLINE_PLANNER_HPP_

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>
#include <cmath>

#define NONE -1e9  // 表示无效值的常量定义
using namespace std;
using namespace Eigen;

using vecD = vector<double>;  // 双精度浮点数向量类型别名
/**
 * @class Spline
 * @brief 一维三次样条插值类
 * @details 实现一维三次样条插值算法，通过给定的控制点生成平滑曲线
 *          支持计算任意参数t处的位置、一阶导数和二阶导数
 */
class Spline
{
private:
    vecD a, b, c, d, w;  // 样条系数向量：a(常数项), b(一次项), c(二次项), d(三次项), w(权重)
    vecD x, y;           // 输入的控制点坐标
    int nx;              // 控制点数量

    // 私有成员函数
    /**
     * @brief 计算样条插值的系数矩阵A
     * @param h 相邻控制点间的距离向量
     * @return 系数矩阵A
     */
    MatrixXd calc_A(vecD h);
    
    /**
     * @brief 计算样条插值的常数向量B
     * @param h 相邻控制点间的距离向量
     * @return 常数向量B
     */
    MatrixXd calc_B(vecD h);
    
    /**
     * @brief 搜索给定参数p对应的区间索引
     * @param p 查询参数
     * @return 区间索引
     */
    int search_index(double p);

public:
    /**
     * @brief 初始化样条插值器
     * @param x_in 输入的x坐标向量
     * @param y_in 输入的y坐标向量
     */
    void init(vecD x_in, vecD y_in);
    
    /**
     * @brief 计算参数t处的函数值
     * @param t 查询参数
     * @return 函数值
     */
    double calc(double t);
    
    /**
     * @brief 计算参数t处的一阶导数
     * @param t 查询参数
     * @return 一阶导数值
     */
    double calcd(double t);
    
    /**
     * @brief 计算参数t处的二阶导数
     * @param t 查询参数
     * @return 二阶导数值
     */
    double calcdd(double t);
};

/**
 * @class Spline2D
 * @brief 二维三次样条插值类
 * @details 实现二维三次样条插值算法，通过给定的二维控制点生成平滑的二维曲线
 *          支持计算任意弧长参数处的位置、曲率和航向角
 */
class Spline2D
{
private:
    Spline sx, sy;  // x和y方向的一维样条插值器
    vecD s;         // 累积弧长向量

    // 私有成员函数
    /**
     * @brief 计算累积弧长向量
     * @param x x坐标向量
     * @param y y坐标向量
     * @return 累积弧长向量
     */
    vecD calc_s(vecD x, vecD y);

public:
    /**
     * @brief 初始化二维样条插值器
     * @param x 输入的x坐标向量
     * @param y 输入的y坐标向量
     */
    void init(vecD x, vecD y);
    
    /**
     * @brief 计算弧长参数s_t处的位置
     * @param s_t 弧长参数
     * @return 二维位置向量
     */
    Vector2d calc_postion(double s_t);
    
    /**
     * @brief 计算弧长参数s_t处的曲率
     * @param s_t 弧长参数
     * @return 曲率值
     */
    double calc_curvature(double s_t);
    
    /**
     * @brief 计算弧长参数s_t处的航向角
     * @param s_t 弧长参数
     * @return 航向角（弧度）
     */
    double calc_yaw(double s_t);
    
    /**
     * @brief 构造函数
     * @param x 输入的x坐标向量
     * @param y 输入的y坐标向量
     */
    Spline2D(vecD x, vecD y) { init(x, y); }
    
    /**
     * @brief 默认构造函数
     */
    Spline2D() {}
    
    /**
     * @brief 获取最后一个弧长值
     * @return 最后一个弧长值
     */
    double get_s_last();
    
    /**
     * @brief 计算弧长参数处的位置（重载版本）
     * @param x 输出x坐标
     * @param y 输出y坐标
     * @param t 弧长参数
     */
    void calc_position(double &x, double &y, double t);
    

};  // end of class

Spline2D calc_spline_course(vecD x, vecD y, vecD &rx, vecD &ry, vecD &ryaw, vecD &rk, double ds);
void printVecD(vecD A);

#endif  // CUBIC_SPLINE_PLANNER_HPP_
