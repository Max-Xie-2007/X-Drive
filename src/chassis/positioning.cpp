#include "chassis/positioning.h"
#include "utils/params.h"
#include "robot-config.h"
using namespace vex;

void updatePosition() {
    while (true) {
        myPosition.update();
        this_thread::sleep_for(cycle.position);
    }
}

Orthogonal::Orthogonal(rotation& rotation_left, rotation& rotation_right,
                       inertial& inertial)
    : rotation_left_(rotation_left), rotation_right_(rotation_right),
      inertial_(inertial) {
    // Timer init
    position_update_timer_.reset();
    last_time_ = 0;
    // Encoder init
    rotation_left_.resetPosition();
    rotation_right_.resetPosition();
    left_dist_ = 0;
    right_dist_ = 0;
    last_left_dist_ = 0;
    last_right_dist_ = 0;
    // Heading init
    heading_unwrapped_ = 0;
    last_heading_unwrapped_ = 0;
    // Position init
    center_pos_ = Point(0, 0);
    last_center_pos_ = Point(0, 0);
    intersection_pos_ = center_pos_ + hardware.odometer.offset;
}

// 重设位置
void Orthogonal::reset(const Point& init_pos, const double& init_heading) {
    // Timer reset
    position_update_timer_.reset();
    last_time_ = 0;
    // Encoder reset
    rotation_left_.resetPosition();
    rotation_right_.resetPosition();
    left_dist_ = 0;
    right_dist_ = 0;
    last_left_dist_ = 0;
    last_right_dist_ = 0;
    // Heading reset
    heading_unwrapped_ = init_heading;
    last_heading_unwrapped_ = init_heading;
    // Position reset
    center_pos_ = init_pos;
    last_center_pos_ = init_pos;
    intersection_pos_ = center_pos_ + hardware.odometer.offset;
}

void Orthogonal::update() {
    // Timer updates
    real_update_cycle_ = position_update_timer_.time() - last_time_;
    last_time_ = position_update_timer_.time();

    // Encoder updates
    last_left_dist_ = left_dist_;
    last_right_dist_ = right_dist_;
    left_dist_ =
        deg2rad(rotation_left_.position(deg)) * hardware.odometer.pos_wheel_radius;
    right_dist_ =
        deg2rad(rotation_right_.position(deg)) * hardware.odometer.pos_wheel_radius;
    if (fabs(left_dist_ - last_left_dist_) < 0.005) left_dist_ = last_left_dist_;
    if (fabs(right_dist_ - last_right_dist_) < 0.005) right_dist_ = last_right_dist_;

    // Heading updates
    last_heading_raw_ = heading_raw_;
    heading_raw_ = inertial_.heading(deg);
    double heading_delta_raw_ = heading_raw_ - last_heading_raw_;
    if (heading_delta_raw_ > 180.0) heading_delta_raw_ -= 360.0;
    if (heading_delta_raw_ < -180.0) heading_delta_raw_ += 360.0;
    heading_delta_ += heading_delta_raw_ * hardware.odometer.IMU_heading_coef;
    if (fabs(heading_delta_) > 0.005) {
        last_heading_unwrapped_ = heading_unwrapped_;
        heading_unwrapped_ += heading_delta_;
        heading_delta_ = 0;
    }

    // Position updates
    double d_l_dist = left_dist_ - last_left_dist_;
    double d_r_dist = right_dist_ - last_right_dist_;
    double d_x = (d_l_dist * sin(hardware.odometer.right_encoder_angle) -
                  d_r_dist * sin(hardware.odometer.left_encoder_angle)) /
                 hardware.odometer.den;
    double d_y = (d_r_dist * cos(hardware.odometer.left_encoder_angle) -
                  d_l_dist * cos(hardware.odometer.right_encoder_angle)) /
                 hardware.odometer.den;
    intersection_pos_ =
        intersection_pos_ +
        Vector(d_x, d_y).rotate((last_heading_unwrapped_ + heading_unwrapped_) / 2.0);
    last_center_pos_ = center_pos_;
    center_pos_ = intersection_pos_ - hardware.odometer.offset.rotate(heading_unwrapped_);

    // Velocity updates
    last_trans_vel_ = trans_vel_;
    trans_vel_ = (center_pos_ - last_center_pos_) / real_update_cycle_;
    trans_vel_ = trans_vel_ * alpha_trans_vel_ + last_trans_vel_ * (1 - alpha_trans_vel_);
    last_angular_vel_ = angular_vel_;
    angular_vel_ = (heading_unwrapped_ - last_heading_unwrapped_) / real_update_cycle_;
    angular_vel_ =
        angular_vel_ * alpha_angular_vel_ + last_angular_vel_ * (1 - alpha_angular_vel_);
}