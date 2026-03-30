#include "auto-functions/motion.h"
#include "ui/ui-base.h"
#include "utils/calc.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>
using namespace vex;

void reach(const Point& target_pos, double target_heading, const Config& cfg) {
    const Config::Reach& reach_cfg = cfg.reach;
    PID2D PID_trans(reach_cfg.PID_trans);
    PID1D PID_rot(reach_cfg.PID_rot);
    target_heading = degNorm(target_heading);
    std::cout << "---------------------reach-----------------------" << std::endl;
    std::cout << "target_pos: (" << target_pos.x_ << ", " << target_pos.y_ << ") "
              << target_heading << std::endl;
    std::cout << "time_limit: " << reach_cfg.time_limit << std::endl;

    // 初始化定时器和PID参数
    timer timeout_timer;
    PID_trans.setErrTol(reach_cfg.trans_tol);
    PID_trans.setDvtTol(reach_cfg.trans_speed_tol);
    PID_trans.setJumpTime(reach_cfg.jump_time);
    PID_trans.reset();
    PID_rot.setErrTol(reach_cfg.rot_tol);
    PID_rot.setDvtTol(reach_cfg.rot_speed_tol);
    PID_rot.setJumpTime(reach_cfg.jump_time);
    PID_rot.reset();
    while (timeout_timer.time(msec) < reach_cfg.time_limit) {
        // 计算当前位置与目标位置的误差
        Point current_pos = myPosition.getCenterPos();
        double current_heading = myPosition.getHeading();
        Vector position_error = target_pos - current_pos;
        double heading_error = degNorm(target_heading - current_heading);
        // 将heading误差调整到[-180, 180]范围，确保走最短方向
        if (heading_error > 180) {
            heading_error -= 360;
        }

        // 计算驱动指令
        PID_trans.update(position_error);
        PID_rot.update(heading_error);
        Vector trans = PID_trans.output_;
        double rotation = PID_rot.output_;
        trans = sat(trans, reach_cfg.max_speed);

        // 执行驱动
        myDrive.setAbsAuton(trans, rotation);

        // 设置跳出循环条件
        if (PID_trans.reached_ && PID_rot.reached_) {
            break;
        }

        this_thread::sleep_for(cycle.auton);
    }
    myDrive.stop(reach_cfg.if_hold ? brakeType::hold : brakeType::coast);

    const Point real_pos = myPosition.getCenterPos();
    std::cout << "current_pos: (" << real_pos.x_ << ", " << real_pos.y_ << ") "
              << myPosition.getHeading() << std::endl;
    std::cout << "elapsed_time: " << timeout_timer.time(msec) << std::endl;
}

template <typename T>
void traceWithGlobalHeading(const T& path, double target_heading, const Config& cfg) {
    const Config::Trace& trace_cfg = cfg.trace;
    PID2D PID_trans(trace_cfg.PID_trans);
    PID1D PID_rot(trace_cfg.PID_rot);
    PID2D PID_back(trace_cfg.PID_back);
    target_heading = degNorm(target_heading);
    std::cout << "-------------traceWithGlobalHeading--------------" << std::endl;
    std::cout << "target_pos: (" << path.end_.x_ << ", " << path.end_.y_ << ") "
              << target_heading << std::endl;
    std::cout << "time_limit: " << trace_cfg.time_limit << std::endl;
    // 初始化定时器和PID参数
    timer timeout_timer;
    PID_trans.setErrTol(trace_cfg.trans_tol);
    PID_trans.setDvtTol(trace_cfg.trans_speed_tol);
    PID_trans.setJumpTime(trace_cfg.jump_time);
    PID_trans.reset();
    PID_rot.setErrTol(trace_cfg.rot_tol);
    PID_rot.setDvtTol(trace_cfg.rot_speed_tol);
    PID_rot.setJumpTime(trace_cfg.jump_time);
    PID_rot.reset();
    while (!trace_cfg.is_terminal || timeout_timer.time(msec) < trace_cfg.time_limit) {
        // 计算当前位置与目标位置的误差
        Point current_pos = myPosition.getCenterPos();
        double current_heading = myPosition.getHeading();
        Vector path_error = current_pos.to(path);
        Circle intersection_circle(current_pos, trace_cfg.look_ahead_dist);
        Point lookahead_target = intersection_circle.findIntersection(path);
        Vector position_error = (std::isnan(lookahead_target.x_))
                                    ? path_error
                                    : lookahead_target - current_pos;
        double heading_error = degNorm(target_heading - current_heading);
        if (heading_error > 180) {
            heading_error -= 360;
        }

        // 计算驱动指令
        PID_trans.update(position_error);
        PID_rot.update(heading_error);
        PID_back.update(path_error);
        Vector translation = PID_trans.output_;
        double rotation = PID_rot.output_;
        Vector back_correction = PID_back.output_;

        Vector trans = translation + back_correction;
        trans = (trans.len() > trace_cfg.max_speed) ? trans.norm() * trace_cfg.max_speed
                                                    : trans;

        // 执行驱动
        myDrive.setAbsAuton(trans, rotation);

        // 设置跳出循环条件
        if (trace_cfg.is_terminal) {
            if (PID_trans.reached_ && PID_rot.reached_) {
                break;
            }
        } else {
            if (position_error.len() < trace_cfg.look_ahead_dist) {
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
    PID2D PID_trans(trace_cfg.PID_trans);
    PID1D PID_rot(trace_cfg.PID_rot);
    PID2D PID_back(trace_cfg.PID_back);
    terminal_target_heading = degNorm(terminal_target_heading);
    heading_offset = degNorm(heading_offset);
    std::cout << "-------------traceWithRelativeHeading--------------" << std::endl;
    std::cout << "target_pos: (" << path.end_.x_ << ", " << path.end_.y_ << ") ";
    if (trace_cfg.is_terminal) {
        std::cout << "terminal_target_heading: " << terminal_target_heading
                  << ", heading_offset: " << heading_offset << std::endl;
    } else {
        std::cout << "heading_offset: " << heading_offset << std::endl;
    }
    std::cout << "time_limit: " << trace_cfg.time_limit << std::endl;
    // 初始化定时器和PID参数
    timer timeout_timer;
    PID_trans.setErrTol(trace_cfg.trans_tol);
    PID_trans.setDvtTol(trace_cfg.trans_speed_tol);
    PID_trans.setJumpTime(trace_cfg.jump_time);
    PID_trans.reset();
    PID_rot.setErrTol(trace_cfg.rot_tol);
    PID_rot.setDvtTol(trace_cfg.rot_speed_tol);
    PID_rot.setJumpTime(trace_cfg.jump_time);
    PID_rot.reset();
    myDrive.setImportantFactor(ROT);
    while (!trace_cfg.is_terminal || timeout_timer.time(msec) < trace_cfg.time_limit) {
        // 计算当前位置与目标位置的误差
        Point current_pos = myPosition.getCenterPos();
        double current_heading = myPosition.getHeading();
        Vector current_velocity = myPosition.getTransVel();
        Vector path_error = current_pos.to(path);
        Circle intersection_circle(current_pos, trace_cfg.look_ahead_dist);
        Point lookahead_target = intersection_circle.findIntersection(path);
        Vector position_error = (std::isnan(lookahead_target.x_))
                                    ? path_error
                                    : lookahead_target - current_pos;
        double heading_error;
        if (position_error.len() < trace_cfg.trans_tol) {
            heading_error = degNorm(terminal_target_heading - current_heading);
        } else if (current_velocity.len() < 0.004) { // 速度过慢时不调整朝向，防止震荡
            heading_error = 0;
        } else {
            heading_error =
                degNorm(current_velocity.angle() + heading_offset - current_heading);
        }
        if (heading_error > 180) {
            heading_error -= 360;
        }

        // 计算驱动指令
        PID_trans.update(position_error);
        PID_rot.update(heading_error);
        PID_back.update(path_error);
        Vector translation = PID_trans.output_;
        double rotation = PID_rot.output_;
        Vector back_correction = PID_back.output_;

        Vector trans = translation + back_correction;
        trans = (trans.len() > trace_cfg.max_speed) ? trans.norm() * trace_cfg.max_speed
                                                    : trans;

        // 执行驱动
        myDrive.setAbsAuton(trans, rotation);

        // 设置跳出循环条件
        if (trace_cfg.is_terminal) {
            if (PID_trans.reached_ && PID_rot.reached_) {
                break;
            }
        } else {
            if (position_error.len() < trace_cfg.look_ahead_dist) {
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