#include "auto-functions/motion.h"
#include "ui/ui-base.h"
#include "utils/calc.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>
using namespace vex;

void reach(const Point& target_position, double target_heading, int time_limit,
           const ReachCfg& reach_cfg, const PIDParam translational_pid,
           const PIDParam angular_pid, ExitConditionOptions translational_ec_opt,
           ExitConditionOptions angular_ec_opt) {
    // log
    std::cout << "---------------------reach-----------------------" << std::endl;
    std::cout << "target_position: (" << target_position.x_ << ", " << target_position.y_
              << ") " << target_heading << std::endl;
    std::cout << "time_limit: " << time_limit << std::endl;

    // init variables
    Vector prev_translational_error = target_position - myPosition.getCenterPos();
    Vector prev_translational_output = myDrive.getTranslationalVelocity();
    double prev_angular_error = degNorm(target_heading - myPosition.getHeading());
    if (prev_angular_error > 180) {
        prev_angular_error -= 360;
    }
    double prev_angular_output = myDrive.getAngularVelocity();
    timer timeout_timer;

    // init PID controllers
    PID2D PID_translational(translational_pid);
    PID1D PID_angular(angular_pid);

    // set drive priority
    myDrive.setPriorFactor(TRANS);

    // init exit conditions
    ExitCondition2D translational_ec(translational_ec_opt);
    ExitCondition1D angular_ec(angular_ec_opt);
    if (reach_cfg.min_translational_speed > 0) translational_ec.setExitOnOvershoot(true);
    if (reach_cfg.min_angular_speed > 0) angular_ec.setExitOnOvershoot(true);
    translational_ec.reset(prev_translational_error);
    angular_ec.reset(prev_angular_error);

    // init debug traces
    std::vector<double> translational_error_x_history;
    std::vector<double> translational_error_y_history;
    std::vector<double> translational_output_x_history;
    std::vector<double> translational_output_y_history;
    std::vector<double> angular_error_history;
    std::vector<double> angular_output_history;

    // loop
    while (timeout_timer.time(msec) < time_limit &&
           !(translational_ec.reached() && angular_ec.reached())) {
        // calculate errors
        const Vector translational_error = target_position - myPosition.getCenterPos();
        const Vector translational_dvt =
            (translational_error - prev_translational_error) / cycle.auton;
        double angular_error = degNorm(target_heading - myPosition.getHeading());
        if (angular_error > 180) {
            angular_error -= 360;
        }
        const double angular_dvt = (angular_error - prev_angular_error) / cycle.auton;

        // calculate outputs
        PID_translational.update(translational_error);
        PID_angular.update(angular_error);
        Vector translational_output = PID_translational.getOutput();
        double angular_output = PID_angular.getOutput();
        if (translational_error.len() > 3.0) {
            translational_output = slew(translational_output, prev_translational_output,
                                        reach_cfg.translational_slew_rate);
        }
        if (fabs(angular_error) > 5.0) {
            angular_output =
                slew(angular_output, prev_angular_output, reach_cfg.angular_slew_rate);
        }
        translational_output =
            sat(translational_output, reach_cfg.max_translational_speed);
        translational_output =
            desat(translational_output, reach_cfg.min_translational_speed);
        angular_output = sat(angular_output, reach_cfg.max_angular_speed);
        angular_output = desat(angular_output, reach_cfg.min_angular_speed);

        // apply outputs
        myDrive.setAbsAuton(translational_output, angular_output);

        // update exit conditions
        translational_ec.update(translational_error, translational_dvt);
        angular_ec.update(angular_error, angular_dvt);

        // update variables
        prev_translational_error = translational_error;
        prev_translational_output = translational_output;
        prev_angular_error = angular_error;
        prev_angular_output = angular_output;

        // store debug traces
        translational_error_x_history.push_back(translational_error.dx_);
        translational_error_y_history.push_back(translational_error.dy_);
        translational_output_x_history.push_back(translational_output.dx_);
        translational_output_y_history.push_back(translational_output.dy_);
        angular_error_history.push_back(angular_error);
        angular_output_history.push_back(angular_output);

        // wait for next cycle
        this_thread::sleep_for(cycle.auton);
    }
    // stop
    if (reach_cfg.is_terminal)
        myDrive.stop(reach_cfg.if_hold ? brakeType::hold : brakeType::coast);

    // log
    std::cout << "current_pos: (" << myPosition.getCenterPos().x_ << ", "
              << myPosition.getCenterPos().y_ << ") " << myPosition.getHeading()
              << std::endl;
    std::cout << "elapsed_time: " << timeout_timer.time(msec) << std::endl;
}

template <typename T>
void traceWithGlobalHeading(const T& path, double target_heading, int time_limit,
                            const TraceCfg& trace_cfg, const PIDParam translational_pid,
                            const PIDParam angular_pid, const PIDParam return_pid,
                            ExitConditionOptions translational_ec_opt,
                            ExitConditionOptions angular_ec_opt) {
    // log
    std::cout << "-------------traceWithGlobalHeading--------------" << std::endl;
    std::cout << "target_position: (" << path.end_.x_ << ", " << path.end_.y_ << ") "
              << target_heading << std::endl;
    std::cout << "time_limit: " << trace_cfg.time_limit << std::endl;

    // init variables
    Vector prev_translational_error = target_position - myPosition.getCenterPos();
    Vector prev_translational_output = myDrive.getTranslationalVelocity();
    double prev_angular_error = degNorm(target_heading - myPosition.getHeading());
    if (prev_angular_error > 180) {
        prev_angular_error -= 360;
    }
    double prev_angular_output = myDrive.getAngularVelocity();
    timer timeout_timer;

    // init PID controllers
    PID2D PID_translational(translational_pid);
    PID1D PID_angular(angular_pid);
    PID2D PID_return(return_pid);

    // set drive priority
    myDrive.setPriorFactor(TRANS);

    // init exit conditions
    ExitCondition2D translational_ec(translational_ec_opt);
    ExitCondition1D angular_ec(angular_ec_opt);
    translational_ec.reset(prev_translational_error);
    angular_ec.reset(prev_angular_error);

    // loop
    while (timeout_timer.time(msec) < time_limit) {
        // 计算当前位置与目标位置的误差
        Point current_pos = myPosition.getCenterPos();
        double current_heading = myPosition.getHeading();
        Vector path_error = current_pos.to(path);
        Circle intersection_circle(current_pos, trace_cfg.look_ahead_dist);
        Point lookahead_target = intersection_circle.findIntersection(path);
        Vector translational_error = (std::isnan(lookahead_target.x_))
                                         ? path_error
                                         : lookahead_target - current_pos;
        double angular_error = degNorm(target_heading - current_heading);
        if (angular_error > 180) {
            angular_error -= 360;
        }

        // 计算驱动指令
        PID_translational.update(translational_error);
        PID_angular.update(angular_error);
        PID_back.update(path_error);
        Vector translation = PID_translational.output_;
        double angular_output = PID_angular.output_;
        Vector back_correction = PID_back.output_;

        Vector translational_output = translation + back_correction;
        translational_output = (translational_output.len() > trace_cfg.max_speed)
                                   ? translational_output.norm() * trace_cfg.max_speed
                                   : translational_output;

        // 执行驱动
        myDrive.setAbsAuton(translational_output, angular_output);

        // 设置跳出循环条件
        if (trace_cfg.is_terminal) {
            if (PID_translational.reached_ && PID_angular.reached_) {
                break;
            }
        } else {
            if (translational_error.len() < trace_cfg.look_ahead_dist) {
                break;
            }
        }

        this_thread::sleep_for(cycle.auton);
    }
    if (trace_cfg.is_terminal) {
        myDrive.stop(trace_cfg.if_hold ? brakeType::hold : brakeType::coast);
    }
    const Point real_pos = myPosition.getCenterPos();
    std::cout << "current_pos: (" << real_pos.x_ << ", " << real_pos.y_ << ") "
              << myPosition.getHeading() << std::endl;
    std::cout << "elapsed_time: " << timeout_timer.time(msec) << std::endl;
}

template <typename T>
void traceWithRelativeHeading(const T& path, double heading_offset,
                              double terminal_target_heading, const Config& cfg) {
    const Config::Trace& trace_cfg = cfg.trace;
    PID2D PID_translational(trace_cfg.PID_translational);
    PID1D PID_angular(trace_cfg.PID_angular);
    PID2D PID_back(trace_cfg.PID_back);
    terminal_target_heading = degNorm(terminal_target_heading);
    heading_offset = degNorm(heading_offset);
    std::cout << "-------------traceWithRelativeHeading--------------" << std::endl;
    std::cout << "target_position: (" << path.end_.x_ << ", " << path.end_.y_ << ") ";
    if (trace_cfg.is_terminal) {
        std::cout << "terminal_target_heading: " << terminal_target_heading
                  << ", heading_offset: " << heading_offset << std::endl;
    } else {
        std::cout << "heading_offset: " << heading_offset << std::endl;
    }
    std::cout << "time_limit: " << trace_cfg.time_limit << std::endl;
    // 初始化定时器和PID参数
    timer timeout_timer;
    PID_translational.setErrTol(trace_cfg.trans_tol);
    PID_translational.setDvtTol(trace_cfg.trans_speed_tol);
    PID_translational.setJumpTime(trace_cfg.jump_time);
    PID_translational.reset();
    PID_angular.setErrTol(trace_cfg.rot_tol);
    PID_angular.setDvtTol(trace_cfg.rot_speed_tol);
    PID_angular.setJumpTime(trace_cfg.jump_time);
    PID_angular.reset();
    myDrive.setPriorFactor(ANGULAR);
    while (!trace_cfg.is_terminal || timeout_timer.time(msec) < trace_cfg.time_limit) {
        // 计算当前位置与目标位置的误差
        Point current_pos = myPosition.getCenterPos();
        double current_heading = myPosition.getHeading();
        Vector current_velocity = myPosition.getTransVel();
        Vector path_error = current_pos.to(path);
        Circle intersection_circle(current_pos, trace_cfg.look_ahead_dist);
        Point lookahead_target = intersection_circle.findIntersection(path);
        Vector translational_error = (std::isnan(lookahead_target.x_))
                                         ? path_error
                                         : lookahead_target - current_pos;
        double angular_error;
        if (translational_error.len() < trace_cfg.trans_tol) {
            angular_error = degNorm(terminal_target_heading - current_heading);
        } else if (current_velocity.len() < 0.004) { // 速度过慢时不调整朝向，防止震荡
            angular_error = 0;
        } else {
            angular_error =
                degNorm(current_velocity.angle() + heading_offset - current_heading);
        }
        if (angular_error > 180) {
            angular_error -= 360;
        }

        // 计算驱动指令
        PID_translational.update(translational_error);
        PID_angular.update(angular_error);
        PID_back.update(path_error);
        Vector translation = PID_translational.output_;
        double angular_output = PID_angular.output_;
        Vector back_correction = PID_back.output_;

        Vector translational_output = translation + back_correction;
        translational_output = (translational_output.len() > trace_cfg.max_speed)
                                   ? translational_output.norm() * trace_cfg.max_speed
                                   : translational_output;

        // 执行驱动
        myDrive.setAbsAuton(translational_output, angular_output);

        // 设置跳出循环条件
        if (trace_cfg.is_terminal) {
            if (PID_translational.reached_ && PID_angular.reached_) {
                break;
            }
        } else {
            if (translational_error.len() < trace_cfg.look_ahead_dist) {
                break;
            }
        }

        this_thread::sleep_for(cycle.auton);
    }
    if (trace_cfg.is_terminal) {
        myDrive.stop(trace_cfg.if_hold ? brakeType::hold : brakeType::coast);
    }
    const Point real_pos = myPosition.getCenterPos();
    std::cout << "current_pos: (" << real_pos.x_ << ", " << real_pos.y_ << ") "
              << myPosition.getHeading() << std::endl;
    std::cout << "elapsed_time: " << timeout_timer.time(msec) << std::endl;
}