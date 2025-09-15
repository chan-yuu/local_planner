/**
 * @file polynomials.cpp
 * @brief 多项式轨迹生成器实现文件
 * @details 实现了五次多项式和四次多项式轨迹生成算法的具体功能
 * @author Frenet Planner Team
 * @date 2024
 */

#include "polynomials.hpp"
#include <eigen3/Eigen/Dense>

using namespace Eigen;

// ==================== QuinticPolynomial类实现 ====================

/**
 * @brief 五次多项式构造函数
 * @param xs_ 起始位置
 * @param vxs_ 起始速度
 * @param axs_ 起始加速度
 * @param xe_ 终止位置
 * @param vxe_ 终止速度
 * @param axe_ 终止加速度
 * @param T_ 时间长度
 * @details 通过求解线性方程组来计算高阶系数a3, a4, a5
 */
QuinticPolynomial::QuinticPolynomial(double xs_, double vxs_, double axs_, double xe_, double vxe_, double axe_, double T_)
{
    // 保存边界条件
    xs = xs_;
    vxs = vxs_;
    axs = axs_;
    xe = xe_;
    vxe = vxe_;
    axe = axe_;
    T = T_;

    // 低阶系数直接由起始条件确定
    a0 = xs;           // 起始位置
    a1 = vxs;          // 起始速度
    a2 = axs / 2.0;    // 起始加速度的一半

    // 构建线性方程组求解高阶系数
    // 方程组基于终止条件：位置、速度、加速度约束
    MatrixXd A = MatrixXd(3, 3);
    A << T * T * T, T * T * T * T, T * T * T * T * T,      // 位置约束系数
         3 * T * T, 4 * T * T * T, 5 * T * T * T * T,      // 速度约束系数
         6 * T, 12 * T * T, 20 * T * T * T;               // 加速度约束系数

    VectorXd B = VectorXd(3);
    B << xe - a0 - a1 * T - a2 * T * T,    // 位置约束右端项
         vxe - a1 - 2 * a2 * T,            // 速度约束右端项
         axe - 2 * a2;                     // 加速度约束右端项

    // 求解线性方程组 AX = B
    VectorXd X = A.colPivHouseholderQr().solve(B);

    // 提取高阶系数
    a3 = X(0);  // 三次项系数
    a4 = X(1);  // 四次项系数
    a5 = X(2);  // 五次项系数
}

/**
 * @brief 计算五次多项式在时刻t的位置值
 * @param t 时间参数
 * @return 位置值 p(t) = a0 + a1*t + a2*t² + a3*t³ + a4*t⁴ + a5*t⁵
 */
double QuinticPolynomial::calc_point(double t)
{
    return a0 + a1 * t + a2 * t * t + a3 * t * t * t + a4 * pow(t, 4) + a5 * pow(t, 5);
}

/**
 * @brief 计算五次多项式在时刻t的一阶导数（速度）
 * @param t 时间参数
 * @return 速度值 v(t) = a1 + 2*a2*t + 3*a3*t² + 4*a4*t³ + 5*a5*t⁴
 */
double QuinticPolynomial::calc_first_derivative(double t)
{
    return a1 + 2.0 * a2 * t + 3.0 * a3 * t * t + 4.0 * a4 * pow(t, 3) + 5.0 * a5 * pow(t, 4);
}

/**
 * @brief 计算五次多项式在时刻t的二阶导数（加速度）
 * @param t 时间参数
 * @return 加速度值 a(t) = 2*a2 + 6*a3*t + 12*a4*t² + 20*a5*t³
 */
double QuinticPolynomial::calc_second_derivative(double t)
{
    return 2.0 * a2 + 6.0 * a3 * t + 12.0 * a4 * t * t + 20.0 * a5 * pow(t, 3);
}

/**
 * @brief 计算五次多项式在时刻t的三阶导数（加加速度/急动度）
 * @param t 时间参数
 * @return 急动度值 j(t) = 6*a3 + 24*a4*t + 60*a5*t²
 */
double QuinticPolynomial::calc_third_derivative(double t)
{
    return 6.0 * a3 + 24.0 * a4 * t + 60.0 * a5 * t * t;
}

// ==================== QuarticPolynomial类实现 ====================

/**
 * @brief 四次多项式构造函数
 * @param xs_ 起始位置
 * @param vxs_ 起始速度
 * @param axs_ 起始加速度
 * @param vxe_ 终止速度
 * @param axe_ 终止加速度
 * @param T_ 时间长度
 * @details 四次多项式只约束速度和加速度的终止条件，不约束终止位置
 */
QuarticPolynomial::QuarticPolynomial(double xs_, double vxs_, double axs_, double vxe_, double axe_, double T_)
{
    // 保存边界条件
    xs = xs_;
    vxs = vxs_;
    axs = axs_;
    vxe = vxe_;
    axe = axe_;
    T = T_;

    // 低阶系数直接由起始条件确定
    a0 = xs;           // 起始位置
    a1 = vxs;          // 起始速度
    a2 = axs / 2.0;    // 起始加速度的一半

    // 构建线性方程组求解高阶系数
    // 方程组基于终止条件：速度、加速度约束
    MatrixXd A = MatrixXd(2, 2);
    A << 3 * T * T, 4 * T * T * T,    // 速度约束系数
         6 * T, 12 * T * T;          // 加速度约束系数

    VectorXd B = VectorXd(2);
    B << vxe - a1 - 2 * a2 * T,      // 速度约束右端项
         axe - 2 * a2;               // 加速度约束右端项

    // 求解线性方程组 AX = B
    VectorXd X = A.colPivHouseholderQr().solve(B);

    // 提取高阶系数
    a3 = X(0);  // 三次项系数
    a4 = X(1);  // 四次项系数
}

/**
 * @brief 计算四次多项式在时刻t的位置值
 * @param t 时间参数
 * @return 位置值 p(t) = a0 + a1*t + a2*t² + a3*t³ + a4*t⁴
 */
double QuarticPolynomial::calc_point(double t)
{
    return a0 + a1 * t + a2 * t * t + a3 * t * t * t + a4 * t * t * t * t;
}

/**
 * @brief 计算四次多项式在时刻t的一阶导数（速度）
 * @param t 时间参数
 * @return 速度值 v(t) = a1 + 2*a2*t + 3*a3*t² + 4*a4*t³
 */
double QuarticPolynomial::calc_first_derivative(double t)
{
    return a1 + 2.0 * a2 * t + 3.0 * a3 * t * t + 4.0 * a4 * t * t * t;
}

/**
 * @brief 计算四次多项式在时刻t的二阶导数（加速度）
 * @param t 时间参数
 * @return 加速度值 a(t) = 2*a2 + 6*a3*t + 12*a4*t²
 */
double QuarticPolynomial::calc_second_derivative(double t)
{
    return 2.0 * a2 + 6.0 * a3 * t + 12.0 * a4 * t * t;
}

/**
 * @brief 计算四次多项式在时刻t的三阶导数（加加速度/急动度）
 * @param t 时间参数
 * @return 急动度值 j(t) = 6*a3 + 24*a4*t
 */
double QuarticPolynomial::calc_third_derivative(double t)
{
    return 6.0 * a3 + 24.0 * a4 * t;
}
