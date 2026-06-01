#include "chassis/positioning.h"
#include "params.h"
#include "robot-config.h"
using namespace vex;

void threadUpdatePosition() {
    wait(100, msec);
    while (true) {
        myPosition.update();
        this_thread::sleep_for(cycle.position);
    }
}

OrthogonalOdometry::OrthogonalOdometry(rotation& rotation_left, rotation& rotation_right,
                                       inertial& inertial)
    : rotation_left_(rotation_left), rotation_right_(rotation_right),
      inertial_(inertial) {
    // Timer init
    position_update_timer_.reset();
    last_time_ = 0;
    // Encoder init
    rotation_left_.resetPosition();
    rotation_right_.resetPosition();
    left_rot_dist_ = 0;
    right_rot_dist_ = 0;
    prev_left_rot_dist_ = 0;
    prev_right_rot_dist_ = 0;
    // Heading init
    heading_raw_ = inertial_.heading(deg);
    last_heading_raw_ = heading_raw_;
    heading_unwrapped_ = 0;
    prev_heading_unwrapped_ = 0;
    // Position init
    center_pos_ = Point(0, 0);
    last_center_pos_ = Point(0, 0);
    intersection_pos_ = center_pos_ + hardware.odometer.offset.rotate(0);
}

void OrthogonalOdometry::reset(const Point& center_pos, double heading) {
    // Timer reset
    position_update_timer_.reset();
    last_time_ = 0;
    // Encoder reset
    rotation_left_.resetPosition();
    rotation_right_.resetPosition();
    left_rot_dist_ = 0;
    right_rot_dist_ = 0;
    prev_left_rot_dist_ = 0;
    prev_right_rot_dist_ = 0;
    // Heading reset
    heading_delta_int_ = 0;
    heading_delta_ = 0;
    heading_raw_ = inertial_.heading(deg);
    last_heading_raw_ = inertial_.heading(deg);
    heading_unwrapped_ = heading;
    prev_heading_unwrapped_ = heading;
    // Position reset
    center_pos_ = center_pos;
    last_center_pos_ = center_pos;
    intersection_pos_ = center_pos_ + hardware.odometer.offset.rotate(heading);
}

void OrthogonalOdometry::reset(const Point& center_pos) {
    // Timer reset
    position_update_timer_.reset();
    last_time_ = 0;
    // Encoder reset
    rotation_left_.resetPosition();
    rotation_right_.resetPosition();
    left_rot_dist_ = 0;
    right_rot_dist_ = 0;
    prev_left_rot_dist_ = 0;
    prev_right_rot_dist_ = 0;
    // Heading reset
    heading_delta_int_ = 0;
    heading_delta_ = 0;
    heading_raw_ = inertial_.heading(deg);
    last_heading_raw_ = inertial_.heading(deg);
    prev_heading_unwrapped_ = heading_unwrapped_;
    // Position reset
    center_pos_ = center_pos;
    last_center_pos_ = center_pos;
    intersection_pos_ = center_pos_ + hardware.odometer.offset.rotate(heading_unwrapped_);
}

void OrthogonalOdometry::update() {
    // Timer updates
    real_update_cycle_ = position_update_timer_.time() - last_time_;
    last_time_ = position_update_timer_.time();

    // Encoder updates
    prev_left_rot_dist_ = left_rot_dist_;
    prev_right_rot_dist_ = right_rot_dist_;
    left_rot_dist_ =
        rotation_left_.position(deg) * hardware.odometer.rotation_dist_per_deg;
    right_rot_dist_ =
        rotation_right_.position(deg) * hardware.odometer.rotation_dist_per_deg;
    if (fabs(left_rot_dist_ - prev_left_rot_dist_) < 0.005)
        left_rot_dist_ = prev_left_rot_dist_;
    if (fabs(right_rot_dist_ - prev_right_rot_dist_) < 0.005)
        right_rot_dist_ = prev_right_rot_dist_;

    // Heading updates
    last_heading_raw_ = heading_raw_;
    heading_raw_ = inertial_.heading(deg);
    double heading_delta_raw_ = heading_raw_ - last_heading_raw_;
    if (heading_delta_raw_ > 180.0) heading_delta_raw_ -= 360.0;
    if (heading_delta_raw_ < -180.0) heading_delta_raw_ += 360.0;
    heading_delta_int_ += heading_delta_raw_ * hardware.odometer.IMU_heading_coef;
    if (fabs(heading_delta_int_) > 0.005) {
        heading_delta_ = heading_delta_int_;
        heading_delta_int_ = 0;
        prev_heading_unwrapped_ = heading_unwrapped_;
        heading_unwrapped_ += heading_delta_;
    } else {
        heading_delta_ = 0;
    }

    // Position updates
    const Vector d_pos = [&]() {
        const double d_l_dist = left_rot_dist_ - prev_left_rot_dist_;
        const double d_r_dist = right_rot_dist_ - prev_right_rot_dist_;
        const double d_x = (d_l_dist * sin(hardware.odometer.right_encoder_angle) -
                            d_r_dist * sin(hardware.odometer.left_encoder_angle)) /
                           hardware.odometer.den;
        const double d_y = (d_r_dist * cos(hardware.odometer.left_encoder_angle) -
                            d_l_dist * cos(hardware.odometer.right_encoder_angle)) /
                           hardware.odometer.den;
        return Vector(d_x, d_y).rotate((prev_heading_unwrapped_ + heading_unwrapped_) /
                                       2.0);
    }();
    intersection_pos_ = intersection_pos_ + d_pos;
    last_center_pos_ = center_pos_;
    center_pos_ = intersection_pos_ - hardware.odometer.offset.rotate(heading_unwrapped_);

    // Velocity updates
    last_trans_vel_ = trans_vel_;
    trans_vel_ = (center_pos_ - last_center_pos_) / real_update_cycle_;
    trans_vel_ = trans_vel_ * alpha_trans_vel_ + last_trans_vel_ * (1 - alpha_trans_vel_);
    last_angular_vel_ = angular_vel_;
    angular_vel_ = (heading_unwrapped_ - prev_heading_unwrapped_) / real_update_cycle_;
    angular_vel_ =
        angular_vel_ * alpha_angular_vel_ + last_angular_vel_ * (1 - alpha_angular_vel_);
}