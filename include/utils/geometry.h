#ifndef GEOMETRY_H
#define GEOMETRY_H

struct Point;
struct Vector;
struct Segment;
struct Arc;

/**
 * @file geometry.h
 * @brief 计算两点间距离
 * @param start 起点
 * @param end 终点
 * @return 距离
 */
double disBetween(const Point& start, const Point& end);

/**
 * @file geometry.h
 * @brief 二维点类，单位为英寸
 */
struct Point {
    double x_ = 0.0;
    double y_ = 0.0;

    Point() = default;
    Point(double x, double y);

    /**
     * @brief 点加向量
     * @param vec 向量
     * @return 新点
     */
    Point operator+(const Vector& vec) const;
    /**
     * @brief 点减点，得到向量
     * @param other 另一个点
     * @return 向量
     */
    Vector operator-(const Point& other) const;
    /**
     * @brief 点减向量，得到另一个点
     * @param vec 向量
     * @return 新点
     */
    Point operator-(const Vector& vec) const;
    /**
     * @brief 判断两点是否相等
     * @param other 另一个点
     * @return 是否相等
     */
    bool operator==(const Point& other) const;
    /**
     * @brief 计算点到线段的距离
     * @param seg 线段
     * @return 距离
     */
    Vector to(const Segment& seg) const;
    /**
     * @brief 计算点到圆弧的距离
     * @param arc 圆弧
     * @return 距离
     */
    Vector to(const Arc& arc) const;
};

/**
 * @file geometry.h
 * @brief 二维向量类，单位为英寸
 */
struct Vector {
    double dx_ = 0.0;
    double dy_ = 0.0;

    Vector() = default;
    /**
     * @brief 利用角度构造单位向量
     * @param angle 角度，单位为度，逆时针为正方向(0..360)
     */
    Vector(double angle);
    /**
     * @brief 利用分量构造向量
     * @param dx x 分量
     * @param dy y 分量
     */
    Vector(double dx, double dy);
    /**
     * @brief 利用两点构造向量
     * @param start 起点
     * @param end 终点
     */
    Vector(const Point& start, const Point& end);
    /**
     * @brief 向量加法
     * @param other 另一个向量
     * @return 新向量
     */
    Vector operator+(const Vector& other) const;
    /**
     * @brief 向量减法
     * @param other 另一个向量
     * @return 新向量
     */
    Vector operator-(const Vector& other) const;
    /**
     * @brief 向量数乘
     * @param scalar 标量
     * @return 新向量
     */
    Vector operator*(const double& scalar) const;
    Vector operator/(const double& scalar) const;
    /**
     * @brief 向量点积
     * @param other 另一个向量
     * @return 点积结果
     */
    double dot(const Vector& other) const;
    /**
     * @brief 向量叉积
     * @param other 另一个向量
     * @return 叉积结果
     */
    double cross(const Vector& other) const;
    /**
     * @brief 计算向量长度
     * @return 向量长度
     */
    double len() const;
    /**
     * @brief 向量归一化(单位向量)
     * @return 归一化后的向量
     */
    Vector norm() const;
    /**
     * @brief 向量旋转
     * @param angle 旋转角度，单位为度，逆时针为正方向(CCW-positive)
     * @return 旋转后的向量
     */
    Vector rotate(double angle) const;
    /**
     * @brief 计算向量角度
     * @return 向量与 x 轴正方向的夹角，单位为度，逆时针为正方向(0..360)
     */
    double angle() const;
    /**
     * @brief 向量投影
     * @param other 另一个向量
     * @return 在另一个向量方向上的投影长度
     */
    double projectOn(const Vector& other) const;
};

/**
 * @file geometry.h
 * @param start, end 起始和终止点
 * @param k, b 斜率和截距
 * @param x0 竖直线段的x坐标
 * @brief 线段类，单位为英寸
 */
struct Segment {
    Point start_ = Point(), end_ = Point();
    double k_ = 0, b_ = 0;
    double x0_ = 0;

    Segment(const Point& start = Point(), const Point& end = Point());
    Segment(double k = 0, double b = 0);
    Segment(double x0);
    Segment(double A, double B, double C);
};

/**
 * @file geometry.h
 * @param center 圆心
 * @param r 半径
 * @param D, E, F 三参
 * @brief 圆类，单位为英寸
 */
struct Circle {
    Point center_ = Point();
    double r_ = 0.0;
    double D_ = 0, E_ = 0, F_ = 0;

    Circle(Point center_ = Point(), double r_ = 0.0);
    Circle(double d_ = 0, double e_ = 0, double f_ = 0);
    /**
     * @brief 计算圆与线段的交点
     * @param seg_ 线段
     * @return 距离上靠近线段终点的交点坐标，若无交点则返回(NAN, NAN)
     */
    Point findIntersection(const Segment& seg_) const;
    /**
     * @brief 计算圆与圆弧的交点
     * @param arc_ 圆弧
     * @return 角度上靠近圆弧终点的交点坐标，若无交点则返回(NAN, NAN)
     */
    Point findIntersection(const Arc& arc_) const;
};

/**
 * @file geometry.h
 * @param center_ 圆心
 * @param r_ 半径
 * @param angle_start_, angle_end_
 * 起始和终止角度，x轴正方向为始边，逆时针为正方向，[0,360)
 * @param start_ 起始点
 * @param end_ 终止点
 * @param clockwise_ 是否按顺时针方向(true 表示顺时针，视角为从上方俯视)
 * @param large_arc_ 是否为优弧
 * @brief 圆弧类，单位为英寸
 */
struct Arc {
    Point center_ = Point();
    double r_ = 0.0;
    double angle_start_ = 0, angle_end_ = 0;
    Point start_ = Point(), end_ = Point();
    bool clockwise_ = false;
    bool large_arc_ = false;
    double D_ = 0, E_ = 0, F_ = 0;
    /**
     * @brief 圆弧构造函数
     * @param r_ 半径
     * @param start_ 起始点
     * @param end_ 终止点
     * @param clockwise_ 是否按顺时针方向(true 表示顺时针，视角为从上方俯视)
     * @param largeArc_ 是否为优弧(大于180‌°‌)
     */
    Arc(const Point& start_ = Point(), const Point& end_ = Point(), double r_ = 0.0,
        bool clockwise_ = true, bool largeArc_ = false);
    /**
     * @brief 圆弧构造函数
     * @param center_ 圆心
     * @param r_ 半径
     * @param angleStart_ 起始角度
     * @param angleEnd_ 终止角度
     * @param clockwise_ 是否按顺时针方向(true 表示顺时针，视角为从上方俯视)
     */
    Arc(const Point& center_, double r_, double angleStart_, double angleEnd_,
        bool clockwise_ = true);
};
#endif // GEOMETRY_H