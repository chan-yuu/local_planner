/**
 * @file polynomials.hpp
 * @brief 多项式轨迹生成器头文件
 * @details 实现了五次多项式和四次多项式轨迹生成算法
 *          用于Frenet坐标系下的纵向和横向轨迹规划
 * @author CYUN
 * @date 2025
 */

#ifndef POLYNOMIALS_HPP_
#define POLYNOMIALS_HPP_

#include <iostream>
#include <vector>
#include <cmath>

using namespace std;

/**
 * @class QuinticPolynomial
 * @brief 五次多项式轨迹生成器
 * @details 根据起始和终止的位置、速度、加速度条件生成五次多项式轨迹
 *          主要用于纵向轨迹规划，能够保证位置、速度、加速度的连续性
 */
class QuinticPolynomial
{
public:
    // 多项式系数：f(t) = a0 + a1*t + a2*t² + a3*t³ + a4*t⁴ + a5*t⁵
    double a0, a1, a2, a3, a4, a5;
    
    // 边界条件参数
    double xs, vxs, axs;  // 起始位置、速度、加速度
    double xe, vxe, axe;  // 终止位置、速度、加速度
    double T;             // 时间长度

    /**
     * @brief 构造函数
     * @param xs_ 起始位置
     * @param vxs_ 起始速度
     * @param axs_ 起始加速度
     * @param xe_ 终止位置
     * @param vxe_ 终止速度
     * @param axe_ 终止加速度
     * @param T_ 时间长度
     */
    QuinticPolynomial(double xs_, double vxs_, double axs_, double xe_, double vxe_, double axe_, double T_);
    
    /**
     * @brief 计算时刻t的位置
     * @param t 时间参数
     * @return 位置值
     */
    double calc_point(double t);
    
    /**
     * @brief 计算时刻t的速度（一阶导数）
     * @param t 时间参数
     * @return 速度值
     */
    double calc_first_derivative(double t);
    
    /**
     * @brief 计算时刻t的加速度（二阶导数）
     * @param t 时间参数
     * @return 加速度值
     */
    double calc_second_derivative(double t);
    
    /**
     * @brief 计算时刻t的加加速度（三阶导数）
     * @param t 时间参数
     * @return 加加速度值
     */
    double calc_third_derivative(double t);
};

/**
 * @class QuarticPolynomial
 * @brief 四次多项式轨迹生成器
 * @details 根据起始的位置、速度、加速度和终止的速度、加速度条件生成四次多项式轨迹
 *          主要用于横向轨迹规划，适用于终止位置不固定的情况
 */
class QuarticPolynomial
{
public:
    // 多项式系数：f(t) = a0 + a1*t + a2*t² + a3*t³ + a4*t⁴
    double a0, a1, a2, a3, a4;
    
    // 边界条件参数
    double xs, vxs, axs;  // 起始位置、速度、加速度
    double vxe, axe;      // 终止速度、加速度
    double T;             // 时间长度

    /**
     * @brief 构造函数
     * @param xs_ 起始位置
     * @param vxs_ 起始速度
     * @param axs_ 起始加速度
     * @param vxe_ 终止速度
     * @param axe_ 终止加速度
     * @param T_ 时间长度
     */
    QuarticPolynomial(double xs_, double vxs_, double axs_, double vxe_, double axe_, double T_);
    
    /**
     * @brief 计算时刻t的位置
     * @param t 时间参数
     * @return 位置值
     */
    double calc_point(double t);
    
    /**
     * @brief 计算时刻t的速度（一阶导数）
     * @param t 时间参数
     * @return 速度值
     */
    double calc_first_derivative(double t);
    
    /**
     * @brief 计算时刻t的加速度（二阶导数）
     * @param t 时间参数
     * @return 加速度值
     */
    double calc_second_derivative(double t);
    
    /**
     * @brief 计算时刻t的加加速度（三阶导数）
     * @param t 时间参数
     * @return 加加速度值
     */
    double calc_third_derivative(double t);
};

#endif
