#ifndef POSITIONING_H
#define POSITIONING_H

#include "utils/calc.h"
#include "utils/geometry.h"
#include "vex.h"
using namespace vex;

// 定位线程函数
void updatePosition();

// 正交定位类
struct OrthogonalOdometry {
  private:
    timer position_update_timer_;
    double real_update_cycle_;
    double last_time_;

    rotation& rotation_left_;
    rotation& rotation_right_;
    inertial& inertial_;

    double left_rot_dist_, last_left_rot_dist_;
    double right_rot_dist_, last_right_rot_dist_;

    double heading_unwrapped_ = 0, last_heading_unwrapped_ = 0;
    double heading_raw_ = 0, last_heading_raw_ = 0;
    double heading_delta_int_ = 0;
    double heading_delta_ = 0;

    double angular_vel_, last_angular_vel_;
    const double alpha_angular_vel_ = 0.4; // 角速度滤波EMA权重系数
    Vector trans_vel_, last_trans_vel_;
    const double alpha_trans_vel_ = 0.4; // 平移速度滤波EMA权重系数

    Point last_center_pos_;
    Point center_pos_;
    Point intersection_pos_;

  public:
    /**
     * @brief 构造定位对象
     * @param rotation_left 左定位轮
     * @param rotation_right 右定位轮
     * @param inertial 陀螺仪
     */
    OrthogonalOdometry(rotation& rotation_left, rotation& rotation_right,
                       inertial& inertial);
    /**
     * @brief 重设定位
     * @param center_pos 待设坐标
     * @param heading 待设航向
     */
    void reset(const Point& center_pos = Point(0, 0), const double& heading = 0);
    /**
     * @brief 更新机器人坐标
     */
    void update();

    Point getCenterPos() const { return center_pos_; }
    Point getIntersectionPos() const { return intersection_pos_; }
    Vector getTranslationalVelocity() const { return trans_vel_; }
    double getAngularVelocity() const { return angular_vel_; }

    double getLeftDist() const { return left_rot_dist_; }
    double getRightDist() const { return right_rot_dist_; }

    double getHeadingUnwrapped() const { return heading_unwrapped_; }
    double getHeading() const { return degNorm(heading_unwrapped_); }
};

#endif // POSITIONING_H