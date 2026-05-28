#include "utils/geometry.h"
#include "utils/calc.h"

#include <cmath>
#include <iostream>
#include <stdlib.h>
double disBetween(const Point& start, const Point& end) {
    return sqrt((start.x_ - end.x_) * (start.x_ - end.x_) +
                (start.y_ - end.y_) * (start.y_ - end.y_));
}

Point::Point(double x, double y) : x_(x), y_(y) {}
Point Point::operator+(const Vector& vec) const {
    return Point(x_ + vec.dx_, y_ + vec.dy_);
}
Vector Point::operator-(const Point& other) const {
    return Vector(x_ - other.x_, y_ - other.y_);
}
Point Point::operator-(const Vector& vec) const {
    return Point(x_ - vec.dx_, y_ - vec.dy_);
}
bool Point::operator==(const Point& other) const {
    return std::abs(x_ - other.x_) < 1e-9 && std::abs(y_ - other.y_) < 1e-9;
}
Vector Point::to(const Segment& seg) const {
    const Vector ap = Vector(seg.start_, *this);
    const Vector ab = Vector(seg.start_, seg.end_);
    const double ab_len_sq = ab.len() * ab.len();
    if (ab_len_sq < 1e-9) return Vector(*this, seg.start_);
    const double t = ap.dot(ab) / ab_len_sq;
    if (t < 0.0)
        return Vector(*this, seg.start_);
    else if (t > 1.0)
        return Vector(*this, seg.end_);
    Point projection = Point(seg.start_.x_ + ab.dx_ * t, seg.start_.y_ + ab.dy_ * t);
    return Vector(*this, projection);
}
Vector Point::to(const Arc& arc) const {
    const Vector oc = Vector(arc.center_, *this);
    const Vector on = oc.norm() * arc.r_;
    const Point projection = Point(arc.center_.x_ + on.dx_, arc.center_.y_ + on.dy_);
    // 检查投影点是否在圆弧上
    const double proj_angle = degNorm360(rad2deg(atan2(on.dy_, on.dx_)));
    bool on_arc = false;
    if (arc.clockwise_) {
        if (arc.angle_end_ > arc.angle_start_) {
            on_arc = (proj_angle >= arc.angle_end_ || proj_angle <= arc.angle_start_);
        } else {
            on_arc = (proj_angle <= arc.angle_start_ && proj_angle >= arc.angle_end_);
        }
    } else {
        if (arc.angle_end_ < arc.angle_start_) {
            on_arc = (proj_angle >= arc.angle_start_ || proj_angle <= arc.angle_end_);
        } else {
            on_arc = (proj_angle >= arc.angle_start_ && proj_angle <= arc.angle_end_);
        }
    }
    if (on_arc) {
        return Vector(*this, projection);
    } else {
        const double dis_to_start = disBetween(*this, arc.start_);
        const double dis_to_end = disBetween(*this, arc.end_);
        if (dis_to_start < dis_to_end) {
            return Vector(*this, arc.start_);
        } else {
            return Vector(*this, arc.end_);
        }
    }
}

// 向量
Vector::Vector(double angle) {
    const double r = deg2rad(angle);
    dx_ = cos(r);
    dy_ = sin(r);
}
Vector::Vector(double dx, double dy) : dx_(dx), dy_(dy) {}
Vector::Vector(const Point& start, const Point& end)
    : dx_(end.x_ - start.x_), dy_(end.y_ - start.y_) {}

Vector Vector::operator+(const Vector& other) const {
    return Vector(dx_ + other.dx_, dy_ + other.dy_);
}
Vector Vector::operator-(const Vector& other) const {
    return Vector(dx_ - other.dx_, dy_ - other.dy_);
}
Vector Vector::operator*(const double& scalar) const {
    return Vector(dx_ * scalar, dy_ * scalar);
}
Vector Vector::operator/(const double& scalar) const {
    return Vector(dx_ / scalar, dy_ / scalar);
}
double Vector::dot(const Vector& other) const {
    return dx_ * other.dx_ + dy_ * other.dy_;
}
double Vector::cross(const Vector& other) const {
    return dx_ * other.dy_ - dy_ * other.dx_;
}
double Vector::len() const { return sqrt(dx_ * dx_ + dy_ * dy_); }
Vector Vector::norm() const {
    double l = len();
    if (l < 1e-9) return Vector(0, 0); // 避免除以零
    return Vector(dx_ / l, dy_ / l);
}
Vector Vector::rotate(double angle) const {
    // Use standard mathematical rotation: positive angle = counter-clockwise
    double r = deg2rad(angle);
    double x = dx_ * cos(r) - dy_ * sin(r);
    double y = dx_ * sin(r) + dy_ * cos(r);
    return Vector(x, y);
}

double Vector::angle() const {
    return (std::abs(dy_) > 1e-9 || std::abs(dx_) > 1e-9)
               ? degNorm360(rad2deg(atan2(dy_, dx_)))
               : NAN;
}
double Vector::projectOn(const Vector& other) const { return this->dot(other.norm()); }

// 线段
Segment::Segment(const Point& start, const Point& end) : start_(start), end_(end) {
    double dx = end.x_ - start.x_;
    double dy = end.y_ - start.y_;
    if (std::abs(dx) < 1e-9) {
        k_ = NAN;
        b_ = NAN;
        x0_ = start.x_;
    } else {
        k_ = dy / dx;
        b_ = start.y_ - k_ * start.x_;
        x0_ = NAN;
    }
}
Segment::Segment(double k, double b) : k_(k), b_(b), x0_(NAN) {}
Segment::Segment(double x0) : k_(NAN), b_(NAN), x0_(x0) {}
Segment::Segment(double A, double B, double C) {
    if (std::abs(B) < 1e-9) {
        x0_ = -C / A;
        k_ = NAN;
        b_ = NAN;
    } else {
        k_ = -A / B;
        b_ = -C / B;
        x0_ = NAN;
    }
}

// 圆
Circle::Circle(Point center, double r) : center_(center), r_(r) {
    D_ = -2 * center_.x_;
    E_ = -2 * center_.y_;
    F_ = center_.x_ * center_.x_ + center_.y_ * center_.y_ - r_ * r_;
}
Circle::Circle(double D, double E, double F) : D_(D), E_(E), F_(F) {
    center_ = Point(-D_ / 2, -E_ / 2);
    r_ = sqrt(D_ * D_ / 4 + E_ * E_ / 4 - F_);
}
Point Circle::findIntersection(const Segment& seg) const {
    // 检查终点是否在圆内
    if (disBetween(seg.end_, center_) < r_) {
        return seg.end_;
    }
    double A, B, C;
    if (!std::isnan(seg.k_)) {
        A = 1 + seg.k_ * seg.k_;
        B = D_ + seg.k_ * (2 * seg.b_ + E_);
        C = F_ + seg.b_ * (seg.b_ + E_);
    } else {
        A = 1;
        B = E_;
        C = F_ + seg.x0_ * (seg.x0_ + D_);
    }
    const double delta = B * B - 4 * A * C;
    if (delta < -1e-7) return Point(NAN, NAN); // 无交点

    const double sqrt_delta = sqrt(std::max(0.0, delta));
    double x1, y1, x2, y2;
    if (!std::isnan(seg.k_)) {
        x1 = (-B + sqrt_delta) / (2 * A);
        y1 = seg.k_ * x1 + seg.b_;
        x2 = (-B - sqrt_delta) / (2 * A);
        y2 = seg.k_ * x2 + seg.b_;
    } else {
        x1 = seg.x0_;
        y1 = (-B + sqrt_delta) / (2 * A);
        x2 = seg.x0_;
        y2 = (-B - sqrt_delta) / (2 * A);
    }
    const Point p1 = Point(x1, y1);
    const Point p2 = Point(x2, y2);
    const double dis1 = disBetween(p1, seg.end_);
    const double dis2 = disBetween(p2, seg.end_);
    return (dis1 < dis2) ? p1 : p2;
}
Point Circle::findIntersection(const Arc& arc) const {
    // 检查终点是否在圆内
    if (disBetween(arc.end_, center_) < r_) {
        return arc.end_;
    }
    Segment _seg(D_ - arc.D_, E_ - arc.E_, F_ - arc.F_); // 两圆公共弦
    // 计算圆与公共弦的交点
    double A, B, C;
    if (!std::isnan(_seg.k_)) {
        A = 1 + _seg.k_ * _seg.k_;
        B = D_ + _seg.k_ * (2 * _seg.b_ + E_);
        C = F_ + _seg.b_ * (_seg.b_ + E_);
    } else {
        A = 1;
        B = E_;
        C = F_ + _seg.x0_ * (_seg.x0_ + D_);
    }
    const double delta = B * B - 4 * A * C;
    if (delta < -1e-7) return Point(NAN, NAN); // 无交点

    const double sqrt_delta = sqrt(std::max(0.0, delta));
    double x1, y1, x2, y2;
    if (!std::isnan(_seg.k_)) {
        x1 = (-B + sqrt_delta) / (2 * A);
        y1 = _seg.k_ * x1 + _seg.b_;
        x2 = (-B - sqrt_delta) / (2 * A);
        y2 = _seg.k_ * x2 + _seg.b_;
    } else {
        x1 = _seg.x0_;
        y1 = (-B + sqrt_delta) / (2 * A);
        x2 = _seg.x0_;
        y2 = (-B - sqrt_delta) / (2 * A);
    }
    const Point p1 = Point(x1, y1);
    const Point p2 = Point(x2, y2);
    const double angle1 =
        degNorm360(rad2deg(atan2(p1.y_ - arc.center_.y_, p1.x_ - arc.center_.x_)));
    const double angle2 =
        degNorm360(rad2deg(atan2(p2.y_ - arc.center_.y_, p2.x_ - arc.center_.x_)));
    bool on_arc1 = false, on_arc2 = false;
    if (arc.clockwise_) {
        if (arc.angle_end_ > arc.angle_start_) {
            on_arc1 = (angle1 >= arc.angle_end_ || angle1 <= arc.angle_start_);
            on_arc2 = (angle2 >= arc.angle_end_ || angle2 <= arc.angle_start_);
        } else {
            on_arc1 = (angle1 <= arc.angle_start_ && angle1 >= arc.angle_end_);
            on_arc2 = (angle2 <= arc.angle_start_ && angle2 >= arc.angle_end_);
        }
    } else {
        if (arc.angle_end_ < arc.angle_start_) {
            on_arc1 = (angle1 >= arc.angle_start_ || angle1 <= arc.angle_end_);
            on_arc2 = (angle2 >= arc.angle_start_ || angle2 <= arc.angle_end_);
        } else {
            on_arc1 = (angle1 >= arc.angle_start_ && angle1 <= arc.angle_end_);
            on_arc2 = (angle2 >= arc.angle_start_ && angle2 <= arc.angle_end_);
        }
    }

    if (on_arc1 && on_arc2) {
        const double angle_dis1 = fabs(degNorm180(angle1 - arc.angle_end_));
        const double angle_dis2 = fabs(degNorm180(angle2 - arc.angle_end_));
        return (angle_dis1 < angle_dis2) ? p1 : p2;
    } else if (on_arc1) {
        return p1;
    } else if (on_arc2) {
        return p2;
    } else {
        return Point(NAN, NAN);
    }
}

// 圆弧
Arc::Arc(const Point& start, const Point& end, double r, bool clockwise, bool large_arc)
    : r_(r), start_(start), end_(end), clockwise_(clockwise), large_arc_(large_arc) {
    // 计算圆心坐标
    const Point mid = Point((start_.x_ + end_.x_) / 2, (start_.y_ + end_.y_) / 2);
    const double q = disBetween(start_, end_) / 2;
    if (r_ < q) r_ = q; // 修复传入极值 r < q 时无解的问题
    const double h = sqrt(r_ * r_ - q * q);
    const double dx = (q > 1e-9) ? (h * (end_.y_ - start_.y_) / (2 * q)) : 0.0;
    const double dy = (q > 1e-9) ? (h * (end_.x_ - start_.x_) / (2 * q)) : 0.0;
    if (clockwise ^ large_arc) { // 异或
        center_ = Point(mid.x_ + dx, mid.y_ - dy);
    } else {
        center_ = Point(mid.x_ - dx, mid.y_ + dy);
    }
    // 计算起点和终点对应的圆心角度数
    angle_start_ =
        degNorm360(rad2deg(atan2(start_.y_ - center_.y_, start_.x_ - center_.x_)));
    angle_end_ = degNorm360(rad2deg(atan2(end_.y_ - center_.y_, end_.x_ - center_.x_)));
    // 计算三参
    D_ = -2 * center_.x_;
    E_ = -2 * center_.y_;
    F_ = center_.x_ * center_.x_ + center_.y_ * center_.y_ - r_ * r_;
}
Arc::Arc(const Point& center, double r, double angle_start, double angle_end,
         bool clockwise)
    : center_(center), r_(r), angle_start_(degNorm360(angle_start)),
      angle_end_(degNorm360(angle_end)), clockwise_(clockwise) {
    // 计算起点和终点坐标
    start_ = Point(center.x_ + r * cos(deg2rad(angle_start_)),
                   center.y_ + r * sin(deg2rad(angle_start_)));
    end_ = Point(center.x_ + r * cos(deg2rad(angle_end_)),
                 center.y_ + r * sin(deg2rad(angle_end_)));
    // 计算三参
    D_ = -2 * center.x_;
    E_ = -2 * center.y_;
    F_ = center.x_ * center.x_ + center.y_ * center.y_ - r * r;
    // 计算是否为大弧
    const double angleDiff = degNorm360(angle_end_ - angle_start_);
    large_arc_ = clockwise_ ^ (angleDiff > 180.0);
}
