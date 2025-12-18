#include "../include/cubic_spline_planner.hpp"
#include <vector>
template <class T>
ostream &operator<<(ostream &os, vector<T> V)
{
	os << "[ ";
	for (auto v : V)
		os << v << " ";
	return os << "]";
}
// #define trace(...) __f(#__VA_ARGS__, __VA_ARGS__)
// template <typename Arg1>
// void __f(const char* name, Arg1&& arg1){cerr << name << " : " << arg1 << endl;}
// template <typename Arg1, typename... Args>
// void __f(const char* names, Arg1&& arg1, Args&&... args){ const char* comma = strchr(names + 1,
// ','); cerr.write(names, comma - names) << " : " << arg1<<" | "; __f(comma+1, args...); }
#define trace(...) 42
// Python NONE equivalents :
// single variable -1e9
// Splines: http://www.cs.cornell.edu/courses/cs4620/2013fa/lectures/16spline-curves.pdf

/************************
For a vector of points (x_i, y_i), it generates cubic splines 
between each (x_i, y_i) and (x_i+1, y_i+1) as :

f_i(x) = y_i(== a_i) + b_i*(x - x_i) + c_i*(x - x_i)^2 + d_i*(x - x_i)^3

where h_i = x_i - x_i-1
************************/

void printVecD(vecD A)
{
	cout << "\n------------Printing the vector----------\n";
	for (unsigned int i = 0; i < A.size(); i++)
		cout << A[i] << "\n";
	cout << "\n-----------------------------------------\n"
		 << endl;
}

/**
 * @brief 计算样条插值的系数矩阵A
 * @param h 相邻控制点间的距离向量
 * @return 系数矩阵A
 * @details 构建三对角矩阵，用于求解二阶导数连续性条件
 */
MatrixXd Spline::calc_A(vecD h)
{
    MatrixXd A = MatrixXd::Zero(nx, nx);
    
    // 边界条件：自然样条（两端二阶导数为0）
    A(0, 0) = 1.0;
    
    // 内部点的连续性条件
    for (int i = 0; i < nx - 1; i++)
    {
        if (i != (nx - 2))
        {
            A(i + 1, i) = h[i];                           // 左邻点系数
            A(i + 1, i + 1) = 2.0 * (h[i] + h[i + 1]);    // 当前点系数
            A(i + 1, i + 2) = h[i + 1];                   // 右邻点系数
        }
    }
    
    // 边界条件设置
    A(0, 1) = 0.0;
    A(nx - 1, nx - 2) = 0.0;
    A(nx - 1, nx - 1) = 1.0;
    
    return A;
}

/**
 * @brief 计算样条插值的常数向量B
 * @param h 相邻控制点间的距离向量
 * @return 常数向量B
 * @details 根据一阶导数连续性条件构建常数向量
 */
MatrixXd Spline::calc_B(vecD h)
{
    MatrixXd B = MatrixXd::Zero(nx, 1);
    
    // 根据一阶导数连续性条件计算常数项
    for (int i = 0; i < nx - 2; i++)
    {
        B(i + 1) = 3.0 * (a[i + 2] - a[i + 1]) / h[i + 1] - 3.0 * (a[i + 1] - a[i]) / h[i];
    }
    
    return B;
}

/**
 * @brief 搜索给定参数p对应的区间索引
 * @param p 查询参数
 * @return 区间索引
 * @details 使用二分查找算法快速定位参数所在的区间
 */
int Spline::search_index(double p)
{
    return upper_bound(x.begin(), x.end(), p) - x.begin() - 1;
}

/**
 * @brief 初始化一维样条插值器
 * @param x_in 输入的x坐标向量（参数）
 * @param y_in 输入的y坐标向量（函数值）
 * @details 通过求解三对角线性方程组来计算样条系数
 */
void Spline::init(vecD x_in, vecD y_in) // calculates the coeffs for splines
{
	x = x_in;
	y = y_in;
	nx = x.size();  // 控制点数量
	
	// 初始化系数向量
	a = y;  // a系数直接等于y值
	b.resize(nx);
	c.resize(nx);
	d.resize(nx);
	w.resize(nx);

	// 计算相邻点间距
	vecD h(nx - 1);
	for (int i = 0; i < nx - 1; i++)
	{
		h[i] = x[i + 1] - x[i];
	}

	// 构建并求解线性方程组 Ac = B
	MatrixXd A = calc_A(h);  // 系数矩阵
	MatrixXd B = calc_B(h);  // 常数向量
	MatrixXd c_eigen = A.colPivHouseholderQr().solve(B);  // 求解c系数
	
	// 将Eigen矩阵转换为std::vector
	double *c_pointer = c_eigen.data();
	c.assign(c_pointer, c_pointer + c_eigen.rows());

	// 计算b和d系数
	for (int i = 0; i < nx - 1; i++)
	{
		d[i] = (c[i + 1] - c[i]) / (3.0 * h[i]);  // 三次项系数
		b[i] = (a[i + 1] - a[i]) / h[i] - h[i] * (c[i + 1] + 2.0 * c[i]) / 3.0;  // 一次项系数
	}
}

/**
 * @brief 计算参数t处的函数值
 * @param t 查询参数
 * @return 函数值，超出范围时返回NONE
 * @details 使用三次多项式公式：f(t) = a + b*dx + c*dx² + d*dx³
 */
double Spline::calc(double t)
{
    // 边界检查
    if (x.size() == 0)
    {
        return NONE;
    }
    else if (t < x[0])
    {
        return NONE;
    }
    else if (t > x[nx - 1])
    {
        return NONE;
    }
    
    int i = search_index(t);  // 找到对应的区间索引
    double dx = t - x[i];     // 计算相对位移
    
    if (i > nx - 1)
    {
        return NONE;
    }
    
    // 三次多项式计算
    double result = a[i] + b[i] * dx + c[i] * dx * dx + d[i] * dx * dx * dx;
    return result;
}

/**
 * @brief 计算参数t处的一阶导数
 * @param t 查询参数
 * @return 一阶导数值，超出范围时返回NONE
 * @details 使用一阶导数公式：f'(t) = b + 2*c*dx + 3*d*dx²
 */
double Spline::calcd(double t)
{
    // 边界检查
    if (t < x[0])
    {
        return NONE;
    }
    else if (t > x[nx - 1])
    {
        return NONE;
    }
    
    int i = search_index(t);  // 找到对应的区间索引
    double dx = t - x[i];     // 计算相对位移
    
    // 一阶导数计算
    double result = b[i] + 2.0 * c[i] * dx + 3.0 * d[i] * dx * dx;
    
    return result;
}

/**
 * @brief 计算参数t处的二阶导数
 * @param t 查询参数
 * @return 二阶导数值，超出范围时返回NONE
 * @details 使用二阶导数公式：f''(t) = 2*c + 6*d*dx
 */
double Spline::calcdd(double t)
{
    // 边界检查
    if (t < x[0])
        return NONE;
    else if (t > x[nx - 1])
        return NONE;

    int i = search_index(t);  // 找到对应的区间索引
    double dx = t - x[i];     // 计算相对位移
    
    // 二阶导数计算
    double result = 2.0 * c[i] + 6.0 * d[i] * dx;

    return result;
}

/****************************
Spline 2D generates the parametric cubic spline 
of x and y as a function of s:

x = f(s)
y = g(s)
****************************/

// ==================== Spline2D类实现 ====================

/**
 * @brief 计算累积弧长向量
 * @param x x坐标向量
 * @param y y坐标向量
 * @return 累积弧长向量
 * @details 计算每个点到起始点的累积弧长距离
 */
vecD Spline2D::calc_s(vecD x, vecD y)
{
    vecD ds;        // 相邻点间距离
    vecD out_s{0};  // 累积弧长，起始点为0
    
    // 计算相邻点间的欧几里得距离
    for (int i = 0; i < (int)x.size() - 1; i++)
    {
        double dx = x[i + 1] - x[i];
        double dy = y[i + 1] - y[i];
        ds.push_back(sqrt(dx * dx + dy * dy));
    }

    // 计算累积弧长
    for (int i = 0; i < (int)ds.size(); i++)
    {
        out_s.push_back(out_s[i] + ds[i]);
    }

    return out_s;
}

/**
 * @brief 初始化二维样条插值器
 * @param x 输入的x坐标向量
 * @param y 输入的y坐标向量
 * @details 计算弧长参数化，并分别对x和y建立样条插值
 */
void Spline2D::init(vecD x, vecD y)
{
    s = calc_s(x, y);  // 计算弧长参数
    sx.init(s, x);     // 建立弧长-x的样条插值
    sy.init(s, y);     // 建立弧长-y的样条插值
}

/**
 * @brief 计算弧长参数s_t处的位置
 * @param s_t 弧长参数
 * @return 二维位置向量
 * @details 通过弧长参数分别计算x和y坐标
 */
Vector2d Spline2D::calc_postion(double s_t)
{
    double x = sx.calc(s_t);  // 计算x坐标
    double y = sy.calc(s_t);  // 计算y坐标
    return Vector2d(x, y);
}

void Spline2D::calc_position(double &x, double &y, double t)
{
	x = sx.calc(t);
	y = sy.calc(t);
}

/**
 * @brief 计算弧长参数s_t处的曲率
 * @param s_t 弧长参数
 * @return 曲率值
 * @details 使用曲率公式：κ = (x'y'' - y'x'') / (x'² + y'²)^(3/2)
 */
double Spline2D::calc_curvature(double s_t)
{
    double dx = sx.calcd(s_t);   // x的一阶导数
    double ddx = sx.calcdd(s_t); // x的二阶导数
    double dy = sy.calcd(s_t);   // y的一阶导数
    double ddy = sy.calcdd(s_t); // y的二阶导数
    
    // 曲率计算公式
    double k = (ddy * dx - ddx * dy) / (pow(dx * dx + dy * dy, 1.5));
    return k;
}



/**
 * @brief 计算弧长参数s_t处的航向角
 * @param s_t 弧长参数
 * @return 航向角（弧度）
 * @details 使用反正切函数计算切线方向角
 */
double Spline2D::calc_yaw(double s_t)
{
    double dx = sx.calcd(s_t);  // x方向的切线分量
    double dy = sy.calcd(s_t);  // y方向的切线分量
    double yaw = atan2(dy, dx); // 计算航向角
    return yaw;
}



double Spline2D::get_s_last()
{
	return s.back();
}

// generates the Spline2D with points along the spline at distance = ds, also returns
// yaw and curvature
Spline2D calc_spline_course(vecD x, vecD y, vecD &rx, vecD &ry, vecD &ryaw, vecD &rk, double ds)
{
	Spline2D sp(x, y);
	vecD s;
	double sRange = sp.get_s_last();
	double sInc = 0;
	while (1)
	{
		if (sInc >= sRange)
		{
			break;
		}
		s.push_back(sInc);
		sInc = sInc + ds;
	}
	// for(i =0 ; i*ds<sRange;i++)  i*ds = sInc
	//   s[i]=i*ds;
	rx.resize(s.size());
	ry.resize(s.size());
	ryaw.resize(s.size());
	rk.resize(s.size());
	for (int i = 0; i < s.size(); i++)
	{
		double ix, iy;
		Vector2d pos = sp.calc_postion(s[i]);
		ix = pos.x();
		iy = pos.y();
		rx[i] = ix;
		ry[i] = iy;
		ryaw[i] = sp.calc_yaw(s[i]);
		rk[i] = sp.calc_curvature(s[i]);
	}
	return sp;
}
