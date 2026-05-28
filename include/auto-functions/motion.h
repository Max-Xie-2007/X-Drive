#ifndef AUTO_FUNCTIONS_MOTION_H
#define AUTO_FUNCTIONS_MOTION_H

#include "chassis/driving.h"
#include "robot-config.h"
#include "utils/exit-condition.h"
#include "utils/geometry.h"
#include "utils/params.h"
#include "utils/pid.h"
#include "vex.h"

using namespace vex;

/********************** CHASSIS CONTROL **********************/

/**
 * @brief 移动机器人到目标位置并调整朝向
 * @param target_pos 目标位置（全局坐标系）
 * @param target_heading 目标朝向（度，CCW-positive）
 * @param time_limit 时间限制（ms）
 * @param reach_cfg 动作配置
 * @param translational_pid 平移PID参数
 * @param angular_pid 旋转PID参数
 * @param translational_ec 平移退出条件
 * @param angular_ec 旋转退出条件
 */
void reach(const Point& target_pos, double target_heading, int time_limit = 2000,
           const ReachCfg& reach_cfg = default_reach_cfg,
           const PIDParam translational_pid = default_reach_translational,
           const PIDParam angular_pid = default_reach_angular,
           ExitConditionOptions translational_ec = default_translational_ec_opt,
           ExitConditionOptions angular_ec = default_angular_ec_opt);

/**
 * @brief 跟踪路径并保持全局固定朝向
 * @param path 路径段
 * @param target_heading 目标朝向（度，CCW-positive）
 * @param time_limit 时间限制（ms）
 * @param trace_cfg 动作配置
 * @param translational_pid 平移PID参数
 * @param angular_pid 旋转PID参数
 * @param return_pid 反向修正PID参数
 * @param translational_ec 平移退出条件
 * @param angular_ec 旋转退出条件
 */
template <typename T>
void traceWithGlobalHeading(
    const T& path, double target_heading, int time_limit = 2000,
    const TraceCfg& trace_cfg = default_trace_cfg,
    const PIDParam translational_pid = default_trace_translational,
    const PIDParam angular_pid = default_trace_angular,
    const PIDParam deviational_pid = default_trace_deviational,
    ExitConditionOptions translational_ec_opt = default_translational_ec_opt,
    ExitConditionOptions angular_ec_opt = default_angular_ec_opt) {

    // log
    std::cout << "-------------traceWithGlobalHeading--------------" << std::endl;
    std::cout << "target_position: (" << path.end_.x_ << ", " << path.end_.y_ << ") "
              << target_heading << std::endl;
    std::cout << "time_limit: " << time_limit << std::endl;

    // init variables
    bool close = false;
    Vector prev_translational_error = [&]() {
        const Circle intersection_circle(myPosition.getCenterPos(),
                                         trace_cfg.look_ahead_dist);
        const Point target_position = intersection_circle.findIntersection(path);
        if (isnan(target_position.x_)) {
            return myPosition.getCenterPos().to(path);
        } else {
            return target_position - myPosition.getCenterPos();
        }
    }();
    Vector prev_translational_output = myPosition.getTranslationalVelocity();
    double prev_angular_error = degNorm180(target_heading - myPosition.getHeading());
    double prev_angular_output = myPosition.getAngularVelocity();
    timer timeout_timer;

    // init PID controllers
    PID2D PID_translational(translational_pid);
    PID1D PID_angular(angular_pid);
    PID2D PID_deviational(deviational_pid);

    // set drive priority
    myDrive.setPriorFactor(TRANS);

    // init exit conditions
    ExitCondition2D translational_ec(translational_ec_opt);
    ExitCondition1D angular_ec(angular_ec_opt);
    if (trace_cfg.min_translational_speed > 0) translational_ec.setExitOnOvershoot(true);
    if (trace_cfg.look_ahead_dist < translational_ec_opt.err_tol_) {
        // prevent look-ahead distance from being smaller than translational error
        // tolerance, which would cause the robot to stop right at the start
        translational_ec.err_tol_ = trace_cfg.look_ahead_dist - 0.5;
    }

    // loop
    while (timeout_timer.time(msec) < time_limit) {
        // find targets
        const Circle intersection_circle(myPosition.getCenterPos(),
                                         trace_cfg.look_ahead_dist);
        const Point target_position = intersection_circle.findIntersection(path);

        // reset exit conditions on close state switch
        if (!close && target_position == path.end_) {
            close = true;
            translational_ec.reset(prev_translational_error);
            angular_ec.reset(prev_angular_error);
        }

        // calculate errors
        const Vector deviational_error = myPosition.getCenterPos().to(path);
        const Vector translational_error = [&]() {
            if (isnan(target_position.x_)) {
                return deviational_error;
            } else {
                return target_position - myPosition.getCenterPos();
            }
        }();
        const Vector translational_dvt =
            (translational_error - prev_translational_error) / cycle.auton;
        double angular_error = degNorm180(target_heading - myPosition.getHeading());
        const double angular_dvt = (angular_error - prev_angular_error) / cycle.auton;

        // update exit conditions
        translational_ec.update(translational_error, translational_dvt);
        angular_ec.update(angular_error, angular_dvt);
        if (translational_ec.reached() && angular_ec.reached()) {
            break;
        }

        // calculate outputs
        PID_deviational.update(deviational_error);
        PID_translational.update(translational_error);
        PID_angular.update(angular_error);
        Vector translational_output =
            PID_translational.getOutput() + PID_deviational.getOutput();
        double angular_output = PID_angular.getOutput();
        if (!close) {
            translational_output = slew(translational_output, prev_translational_output,
                                        trace_cfg.translational_slew_rate);
            angular_output =
                slew(angular_output, prev_angular_output, trace_cfg.angular_slew_rate);
        }
        translational_output =
            sat(translational_output, trace_cfg.max_translational_speed);
        angular_output = sat(angular_output, trace_cfg.max_angular_speed);
        // translational_output =
        //     desat(translational_output, trace_cfg.min_translational_speed);

        // apply outputs
        myDrive.setAbsAuton(translational_output, angular_output);

        // update variables
        prev_translational_error = translational_error;
        prev_translational_output = translational_output;
        prev_angular_error = angular_error;
        prev_angular_output = angular_output;

        // wait for next cycle
        this_thread::sleep_for(cycle.auton);
    }
    // stop
    if (trace_cfg.is_terminal) {
        myDrive.stop(trace_cfg.if_hold ? brakeType::hold : brakeType::coast);
    }

    // log
    std::cout << "current_pos: (" << myPosition.getCenterPos().x_ << ", "
              << myPosition.getCenterPos().y_ << ") " << myPosition.getHeading()
              << std::endl;
    std::cout << "elapsed_time: " << timeout_timer.time(msec) << std::endl;
}

/**
 * @brief 跟踪路径并保持相对朝向（相对于机器人速度方向）
 * @param path 路径段
 * @param terminal_target_heading 最终目标朝向（度，CCW-positive），!is_terminal时有效
 * @param target_heading_offset 朝向偏移（度，CCW-positive），相对于机器人速度方向的偏移
 * @param time_limit 时间限制（ms）
 * @param trace_cfg 动作配置
 * @param translational_pid 平移PID参数
 * @param angular_pid 旋转PID参数
 * @param return_pid 反向修正PID参数
 * @param translational_ec 平移退出条件
 * @param angular_ec 旋转退出条件
 */
template <typename T>
void traceWithRelativeHeading(
    const T& path, double terminal_target_heading, double target_heading_offset = 0,
    int time_limit = 2000, const TraceCfg& trace_cfg = default_trace_cfg,
    const PIDParam translational_pid = default_trace_translational,
    const PIDParam angular_pid = default_trace_angular,
    const PIDParam deviational_pid = default_trace_deviational,
    ExitConditionOptions translational_ec_opt = default_translational_ec_opt,
    ExitConditionOptions angular_ec_opt = default_angular_ec_opt) {

    // log
    std::cout << "-------------traceWithRelativeHeading--------------" << std::endl;
    std::cout << "target_position: (" << path.end_.x_ << ", " << path.end_.y_ << ") "
              << target_heading_offset << std::endl;
    std::cout << "time_limit: " << time_limit << std::endl;

    // init variables
    bool close = false;
    Vector prev_translational_error = [&]() {
        const Circle intersection_circle(myPosition.getCenterPos(),
                                         trace_cfg.look_ahead_dist);
        const Point target_position = intersection_circle.findIntersection(path);
        if (isnan(target_position.x_)) {
            return myPosition.getCenterPos().to(path);
        } else {
            return target_position - myPosition.getCenterPos();
        }
    }();
    Vector prev_translational_output = myPosition.getTranslationalVelocity();
    double prev_angular_error = [&]() {
        if (myPosition.getTranslationalVelocity().len() < 1e-4) {
            return 0;
        } else {
            return myPosition.getTranslationalVelocity().angle() + target_heading_offset -
                   myPosition.getHeading();
        }
    }();
    double prev_angular_output = myPosition.getAngularVelocity();
    timer timeout_timer;

    // init PID controllers
    PID2D PID_translational(translational_pid);
    PID1D PID_angular(angular_pid);
    PID2D PID_deviational(deviational_pid);

    // set drive priority
    myDrive.setPriorFactor(TRANS);

    // init exit conditions
    ExitCondition2D translational_ec(translational_ec_opt);
    ExitCondition1D angular_ec(angular_ec_opt);
    if (trace_cfg.min_translational_speed > 0) translational_ec.setExitOnOvershoot(true);
    if (trace_cfg.look_ahead_dist < translational_ec_opt.err_tol_) {
        // prevent look-ahead distance from being smaller than translational error
        // tolerance, which would cause the robot to stop right at the start
        translational_ec.err_tol_ = trace_cfg.look_ahead_dist - 0.5;
    }

    // loop
    while (timeout_timer.time(msec) < time_limit) {
        // find targets
        const Circle intersection_circle(myPosition.getCenterPos(),
                                         trace_cfg.look_ahead_dist);
        const Point target_position = intersection_circle.findIntersection(path);
        double target_heading = [&]() {
            if (close && !trace_cfg.is_terminal) {
                return terminal_target_heading;
            } else if (myPosition.getTranslationalVelocity().len() < 1e-4) {
                return myPosition.getHeading();
            } else {
                return myPosition.getTranslationalVelocity().angle() +
                       target_heading_offset;
            }
        }();

        // reset exit conditions on close state switch
        if (!close && target_position == path.end_) {
            close = true;
            translational_ec.reset(prev_translational_error);
            angular_ec.reset(prev_angular_error);
        }

        // calculate errors
        const Vector deviational_error = myPosition.getCenterPos().to(path);
        const Vector translational_error = [&]() {
            if (isnan(target_position.x_)) {
                return deviational_error;
            } else {
                return target_position - myPosition.getCenterPos();
            }
        }();
        const Vector translational_dvt =
            (translational_error - prev_translational_error) / cycle.auton;
        double angular_error = degNorm180(target_heading - myPosition.getHeading());
        if (angular_error > 180) {
            angular_error -= 360;
        }
        const double angular_dvt = (angular_error - prev_angular_error) / cycle.auton;

        // update exit conditions
        translational_ec.update(translational_error, translational_dvt);
        angular_ec.update(angular_error, angular_dvt);
        if (translational_ec.reached() && angular_ec.reached()) {
            break;
        }
        // calculate outputs
        PID_deviational.update(deviational_error);
        PID_translational.update(translational_error);
        PID_angular.update(angular_error);
        Vector translational_output =
            PID_translational.getOutput() + PID_deviational.getOutput();
        double angular_output = PID_angular.getOutput();
        if (!close) {
            translational_output = slew(translational_output, prev_translational_output,
                                        trace_cfg.translational_slew_rate);
            angular_output =
                slew(angular_output, prev_angular_output, trace_cfg.angular_slew_rate);
        }
        translational_output =
            sat(translational_output, trace_cfg.max_translational_speed);
        angular_output = sat(angular_output, trace_cfg.max_angular_speed);
        // translational_output =
        //     desat(translational_output, trace_cfg.min_translational_speed);

        // apply outputs
        myDrive.setAbsAuton(translational_output, angular_output);

        // update variables
        prev_translational_error = translational_error;
        prev_translational_output = translational_output;
        prev_angular_error = angular_error;
        prev_angular_output = angular_output;

        // wait for next cycle
        this_thread::sleep_for(cycle.auton);
    }
    // stop
    if (trace_cfg.is_terminal) {
        myDrive.stop(trace_cfg.if_hold ? brakeType::hold : brakeType::coast);
    }

    // log
    std::cout << "current_pos: (" << myPosition.getCenterPos().x_ << ", "
              << myPosition.getCenterPos().y_ << ") " << myPosition.getHeading()
              << std::endl;
    std::cout << "elapsed_time: " << timeout_timer.time(msec) << std::endl;
}

#endif // AUTO_FUNCTIONS_MOTION_H