#include "auto-functions/motion.h"
#include "ui/base.h"
#include "utils/calc.h"
#include <algorithm>
#include <iostream>
#include <vector>
using namespace vex;

#ifdef XDRIVE
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
        Vector position_error =
            (lookahead_target.x_ == NAN) ? path_error : lookahead_target - current_pos;
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
        Vector position_error =
            (lookahead_target.x_ == NAN) ? path_error : lookahead_target - current_pos;
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
#endif

#ifdef STRAIGHT
double pidForward(double target_distance, double target_heading, const Config& cfg) {
    const Config::Forward& forward_cfg = cfg.forward;
    PID1D PID_fb(forward_cfg.PID_fb);
    PID1D PID_rot(forward_cfg.PID_rot);
    target_heading = degNorm(target_heading);
    std::cout << "---------------------pidForward-----------------------" << std::endl;
    std::cout << "target_distance: " << target_distance
              << " inch, target_heading: " << target_heading << std::endl;
    std::cout << "time_limit: " << forward_cfg.time_limit << std::endl;

    // 1. Initialize timer, odometry baseline and PID configs
    timer timeout_timer;
    timer heading_err_jump_timer;
    myDrive.resetPos();
    myDrive.setImportantFactor(ROT);
    PID_fb.setErrTol(forward_cfg.fb_tol);
    PID_fb.setDvtTol(forward_cfg.fb_speed_tol);
    PID_fb.setJumpTime(forward_cfg.jump_time);
    PID_fb.reset();
    PID_rot.setErrTol(forward_cfg.rot_tol);
    PID_rot.setDvtTol(forward_cfg.rot_speed_tol);
    PID_rot.setJumpTime(forward_cfg.jump_time);
    PID_rot.reset();
    std::vector<double> fb_history;
    std::vector<double> rot_history;
    std::vector<double> fb_output_history;
    std::vector<double> rot_output_history;
    while (timeout_timer.time(msec) < forward_cfg.time_limit) {
        // 2. Read state and compute distance/heading errors
        double current_distance = myDrive.getDisAvg(); // inch
        double current_heading = myPosition.getHeading();
        double distance_error = target_distance - current_distance;
        double heading_error = degNorm(target_heading - current_heading);
        if (heading_error > 180) {
            heading_error -= 360;
        }

        // 3. Update PID outputs
        PID_fb.update(distance_error);
        PID_rot.update(heading_error);

        // 4. Apply speed limits and heading-priority gate
        double fb_speed = sat(sat(PID_fb.output_, forward_cfg.max_speed),
                              heading_err_jump_timer.time() * 0.5);
        double rot_speed = PID_rot.output_;
        if (fabs(heading_error) > 7) { // 角度误差过大时优先调整朝向，防止走偏和震荡
            heading_err_jump_timer.reset();
        }
        if (heading_err_jump_timer.time(msec) <= 100) {
            fb_speed = 0;
        }

        // 5. Send command and check exit conditions
        myDrive.setAuton(fb_speed, rot_speed);
        if (PID_fb.reached_ && PID_rot.reached_) {
            break;
        }

        // 6. Store debug traces
        fb_history.push_back(current_distance);
        rot_history.push_back(current_heading);
        fb_output_history.push_back(fb_speed);
        rot_output_history.push_back(rot_speed);

        this_thread::sleep_for(cycle.auton);
    }

    // 7. Stop chassis and print summary
    if (forward_cfg.is_terminal) {
        myDrive.stop(forward_cfg.if_hold ? brakeType::hold : brakeType::coast);
    }
    std::cout << "current_distance: " << myDrive.getDisAvg()
              << " inch, current_heading: " << myPosition.getHeading() << std::endl;
    std::cout << "elapsed_time: " << timeout_timer.time(msec) << std::endl;
    if (current_mode == PID_DEBUG) {
        const size_t max_rows = 30;
        const size_t step = (std::max<size_t>)(1, fb_history.size() / max_rows);
        std::cout << " FB | ROT | FB OUT | ROT OUT " << std::endl;
        std::cout << "samples: " << fb_history.size() << ", print every " << step
                  << " row(s)" << std::endl;
        for (size_t i = 0; i < fb_history.size(); i += step) {
            std::cout << fb_history[i] << " | " << rot_history[i] << " | "
                      << fb_output_history[i] << " | " << rot_output_history[i]
                      << std::endl;
        }
    }
    return myDrive.getPosAvg();
}

void turnTo(double target_heading, const Config& cfg) {
    const Config::Turn& turn_cfg = cfg.turn;
    PID1D PID_rot(turn_cfg.PID_rot);
    PID1D PID_adj(turn_cfg.PID_fb);
    target_heading = degNorm(target_heading);
    std::cout << "---------------------turnTo-----------------------" << std::endl;
    std::cout << "target_heading: " << target_heading << std::endl;
    std::cout << "time_limit: " << turn_cfg.time_limit << std::endl;

    // 1. Initialize timer, wheel baseline and PID configs
    timer timeout_timer;
    myDrive.resetPos();
    PID_rot.setErrTol(turn_cfg.rot_tol);
    PID_rot.setDvtTol(turn_cfg.rot_speed_tol);
    PID_rot.setJumpTime(turn_cfg.jump_time);
    PID_rot.reset();
    PID_adj.setErrTol(turn_cfg.adj_tol);
    PID_adj.setDvtTol(turn_cfg.adj_speed_tol);
    PID_adj.setJumpTime(turn_cfg.jump_time);
    PID_adj.reset();
    myDrive.setImportantFactor(TRANS);
    std::vector<double> rot_history;
    std::vector<double> diff_history;
    std::vector<double> rot_output_history;
    std::vector<double> adj_output_history;
    while (timeout_timer.time(msec) < turn_cfg.time_limit) {
        // 2. Read state and compute heading/diff errors
        double current_heading = myPosition.getHeading();
        double heading_error = degNorm(target_heading - current_heading);
        if (heading_error > 180) {
            heading_error -= 360;
        }
        // Left+right sum, used to suppress translational drift while turning
        double diff = myDrive.getDisAvg();

        // 3. Update PID outputs
        PID_rot.update(heading_error);
        PID_adj.update(diff);

        // 4. Apply ramped cap to rot speed and generate drift correction
        double rot_speed =
            sat(PID_rot.output_, (std::min)(static_cast<double>(turn_cfg.max_speed),
                                            timeout_timer.time() * 0.5));
        double diff_correction = -PID_adj.output_;

        // 5. Send command and check exit conditions
        myDrive.setAuton(diff_correction, rot_speed);

        if (PID_rot.reached_ && PID_adj.reached_) {
            break;
        }

        // 6. Store debug traces
        rot_history.push_back(current_heading);
        diff_history.push_back(diff);
        rot_output_history.push_back(rot_speed);
        adj_output_history.push_back(diff_correction);

        this_thread::sleep_for(cycle.auton);
    }

    // 7. Stop chassis and print summary
    if (turn_cfg.is_terminal) {
        myDrive.stop(turn_cfg.if_hold ? brakeType::hold : brakeType::coast);
    }
    std::cout << "current_heading: " << myPosition.getHeading() << std::endl;
    std::cout << "elapsed_time: " << timeout_timer.time(msec) << std::endl;
    if (current_mode == PID_DEBUG) {
        const size_t max_rows = 30;
        const size_t step = (std::max<size_t>)(1, rot_history.size() / max_rows);
        std::cout << "ROT | DIFF | ROT OUT | ADJ OUT " << std::endl;
        std::cout << "samples: " << rot_history.size() << ", print every " << step
                  << " row(s)" << std::endl;
        for (size_t i = 0; i < rot_history.size(); i += step) {
            std::cout << rot_history[i] << " | " << diff_history[i] << " | "
                      << rot_output_history[i] << " | " << adj_output_history[i]
                      << std::endl;
        }
    }
}
void moveToPose(const Point& target_pos, double target_heading, bool reversed,
                const Config& cfg) {
    const Config::Pose& pose_cfg = cfg.pose;
    setParamProfile(PROFILE_POSE_FAR);
    PID1D PID_fb(pose_cfg.PID_fb_far);
    PID1D PID_rot(pose_cfg.PID_rot_far);
    double max_speed = pose_cfg.max_speed;
    const double close_dis_th = pose_cfg.close_dis_th;
    const double close_heading_th = pose_cfg.close_heading_th;
    const double leading_factor = pose_cfg.leading_factor;
    target_heading = degNorm(target_heading);
    std::cout << "---------------------moveToPose-----------------------" << std::endl;
    std::cout << "target_pos: (" << target_pos.x_ << ", " << target_pos.y_ << ") "
              << target_heading << std::endl;
    std::cout << "time_limit: " << pose_cfg.time_limit << std::endl;

    timer timeout_timer;
    timer fb_soft_start_timer;
    PID_fb.setErrTol(pose_cfg.fb_tol);
    PID_fb.setDvtTol(pose_cfg.fb_speed_tol);
    PID_fb.setJumpTime(pose_cfg.jump_time);
    PID_fb.reset();
    PID_rot.setErrTol(pose_cfg.rot_tol);
    PID_rot.setDvtTol(pose_cfg.rot_speed_tol);
    PID_rot.setJumpTime(pose_cfg.jump_time);
    PID_rot.reset();
    myDrive.setImportantFactor(ROT);
    enum { FAR, CLOSE_BAD, CLOSE_GOOD } close_condition = FAR;
    enum ExitReason { EXIT_TIMEOUT, EXIT_PID_REACHED, EXIT_NON_TERMINAL_CLOSE };
    ExitReason exit_reason = EXIT_TIMEOUT;
    Point carrot_point = target_pos;
    const double terminal_target_control_heading =
        reversed ? degNorm(target_heading + 180) : target_heading;
    const Vector t_hat(cos(deg2rad(terminal_target_control_heading)),
                       sin(deg2rad(terminal_target_control_heading)));
    const Vector n_hat(-t_hat.dy_, t_hat.dx_); // t_hat.rotate(90)
    const double initial_side = (myPosition.getCenterPos() - target_pos).dot(n_hat);
    const bool initial_side_valid = fabs(initial_side) > 1e-6;
    double prev_fb_cmd = 0;
    double prev_rot_cmd = 0;
    std::vector<double> t_history;
    std::vector<double> distance_history;
    std::vector<double> heading_to_target_history;
    std::vector<double> rot_err_history;
    std::vector<double> fb_dvt_history;
    std::vector<double> rot_dvt_history;
    std::vector<int> pid_fb_reached_history;
    std::vector<int> pid_rot_reached_history;
    double prev_fb_err_debug = 0;
    double prev_rot_err_debug = 0;
    bool first_debug_sample = true;
    while (timeout_timer.time(msec) < pose_cfg.time_limit) {
        // 1. Find if close
        const Point current_pos = myPosition.getCenterPos();
        const double current_heading = myPosition.getHeading();
        const double control_heading =
            reversed ? degNorm(current_heading + 180) : current_heading;
        const double target_distance = disBetween(current_pos, target_pos);
        // detect if the robot has crossed the tangent
        const double current_side = (current_pos - target_pos).dot(n_hat);
        const bool crossed_tangent =
            initial_side_valid && (current_side * initial_side < 0.0);
        // calculate heading to target pose
        double heading_to_target =
            degNorm(terminal_target_control_heading - control_heading);
        if (heading_to_target > 180) {
            heading_to_target -= 360;
        }
        // bad close condition
        if (crossed_tangent && fabs(heading_to_target) < close_heading_th &&
            close_condition == FAR) {
            close_condition = CLOSE_BAD;
            setParamProfile(PROFILE_POSE_CLOSE_BAD);
            max_speed = (std::max)(30.0, max_speed / 2.0);
            PID_fb.setParam(pose_cfg.PID_fb_close_bad);
            PID_rot.setParam(pose_cfg.PID_rot_close_bad);
            Controller.rumble("..");
        }
        // good close condition
        if (!crossed_tangent && target_distance < close_dis_th &&
            fabs(heading_to_target) < close_heading_th && close_condition == FAR) {
            close_condition = CLOSE_GOOD;
            setParamProfile(PROFILE_POSE_CLOSE_GOOD);
            max_speed = (std::max)(45.0, max_speed / 2.0);
            PID_fb.setParam(pose_cfg.PID_fb_close_good);
            PID_rot.setParam(pose_cfg.PID_rot_close_good);
            Controller.rumble(".");
        }

        // 2. Apply close condition to carrot point
        if (close_condition == FAR) {
            carrot_point =
                target_pos - Vector(leading_factor * disBetween(current_pos, target_pos) *
                                        cos(deg2rad(terminal_target_control_heading)),
                                    leading_factor * disBetween(current_pos, target_pos) *
                                        sin(deg2rad(terminal_target_control_heading)));
        } else {
            carrot_point = target_pos;
        }

        // 3. Calculate errors and calculate raw control outputs by pid controller
        const double target_control_heading = close_condition == FAR
                                                  ? (carrot_point - current_pos).angle()
                                                  : terminal_target_control_heading;
        double rot_err = target_control_heading - control_heading;
        // when close put distance to tangent line as addition to heading error
        if (close_condition != FAR && target_distance > close_dis_th / 4.0) {
            const double distance_to_tangent_line = (current_pos - target_pos).dot(n_hat);
            const double adj = -sat(
                1.5 * rad2deg(asin(distance_to_tangent_line / close_dis_th)),
                close_heading_th / 2.0); // n_hat points to the left of terminal heading
            if (pose_cfg.is_terminal) rot_err += adj;
            // std::cout << "adj: " << adj << std::endl;
        }
        rot_err = degNorm(rot_err);
        if (rot_err > 180) {
            rot_err -= 360;
        }

        double fb_err = close_condition == FAR
                            ? disBetween(current_pos, carrot_point)
                            : fabs((carrot_point - current_pos).projectOn(t_hat));
        if (reversed) {
            fb_err = -fb_err;
        }
        if (close_condition != FAR) {
            const int sign = sgn((target_pos - current_pos).dot(t_hat));
            fb_err *= sign;
        }
        PID_fb.update(fb_err);
        PID_rot.update(rot_err);
        double fb_speed = PID_fb.output_;
        double rot_speed = PID_rot.output_;

        // 4. Side slip prevention by cutting fb speed
        if (fabs(rot_err) > (close_condition == FAR ? 20.0 : 15.0)) {
            // clear fb_speed when rot_err is too large
            fb_speed = 0;
        } else {
            // cap fb_speed
            const double fb_cap =
                (std::max)(10.0, close_condition == FAR
                                     ? max_speed *
                                           min(1.1 - pow(fabs(rot_err) / 20.0, 0.35), 1.0)
                                     : max_speed);
            fb_speed = sat(fb_speed, fb_cap);
        }

        // 5. Smooth command edges
        const double fb_step = close_condition == FAR ? 3.0 : 10.0;
        const double rot_step = close_condition == FAR ? 10.0 : 3.0;
        fb_speed = prev_fb_cmd + sat(fb_speed - prev_fb_cmd, fb_step);
        rot_speed = prev_rot_cmd + sat(rot_speed - prev_rot_cmd, rot_step);
        prev_fb_cmd = fb_speed;
        prev_rot_cmd = rot_speed;
        fb_speed = sat(fb_speed, timeout_timer.time() * 0.5);

        // set deadbands for fb_speed
        fb_speed = desat(fb_speed, pose_cfg.fb_speed_min);

        myDrive.setAuton(fb_speed, rot_speed);

        // 6. Debug info storage: keep loop-exit related probes
        const double fb_dvt =
            first_debug_sample ? 0 : (fb_err - prev_fb_err_debug) / cycle.auton;
        const double rot_dvt =
            first_debug_sample ? 0 : (rot_err - prev_rot_err_debug) / cycle.auton;
        prev_fb_err_debug = fb_err;
        prev_rot_err_debug = rot_err;
        first_debug_sample = false;
        t_history.push_back(timeout_timer.time(msec));
        distance_history.push_back(target_distance);
        heading_to_target_history.push_back(heading_to_target);
        rot_err_history.push_back(rot_err);
        fb_dvt_history.push_back(fb_dvt);
        rot_dvt_history.push_back(rot_dvt);
        pid_fb_reached_history.push_back(PID_fb.reached_ ? 1 : 0);
        pid_rot_reached_history.push_back(PID_rot.reached_ ? 1 : 0);

        // 7. Check exit conditions
        if (PID_fb.reached_ && PID_rot.reached_ && close_condition != FAR) {
            exit_reason = EXIT_PID_REACHED;
            break;
        }
        if (!pose_cfg.is_terminal && (target_distance < close_dis_th * 0.5)) {
            exit_reason = EXIT_NON_TERMINAL_CLOSE;
            break;
        }
        this_thread::sleep_for(cycle.auton);
    }
    if (pose_cfg.is_terminal)
        myDrive.stop(pose_cfg.if_hold ? brakeType::hold : brakeType::coast);

    const Point real_pos = myPosition.getCenterPos();
    const double real_heading = myPosition.getHeading();
    std::cout << "current_pos: (" << real_pos.x_ << ", " << real_pos.y_ << ") "
              << real_heading << std::endl;
    std::cout << "elapsed_time: " << timeout_timer.time(msec) << std::endl;

    // if (current_mode == PID_DEBUG) {
    //     const size_t max_rows = 20;
    //     const size_t step = (std::max<size_t>)(1, t_history.size() / max_rows);
    //     const char* exit_reason_str =
    //         (exit_reason == EXIT_PID_REACHED)
    //             ? "PID_REACHED"
    //             : (exit_reason == EXIT_NON_TERMINAL_CLOSE ? "NON_TERMINAL_CLOSE"
    //                                                       : "TIMEOUT");
    //     std::cout << "exit_reason: " << exit_reason_str << std::endl;
    //     std::cout
    //         << "T | DIS | HEAD_TO_TARGET | ROT_ERR | FB_DVT | ROT_DVT | FB_OK | ROT_OK"
    //         << std::endl;
    //     std::cout << "samples: " << t_history.size() << ", print every " << step
    //               << " row(s)" << std::endl;
    //     for (size_t i = 0; i < t_history.size(); i += step) {
    //         std::cout << t_history[i] << " | " << distance_history[i] << " | "
    //                   << heading_to_target_history[i] << " | " << rot_err_history[i]
    //                   << " | " << fb_dvt_history[i] << " | " << rot_dvt_history[i]
    //                   << " | " << pid_fb_reached_history[i] << " | "
    //                   << pid_rot_reached_history[i] << std::endl;
    //     }
    // }
}
#endif

#ifdef XDRIVE

void autoPark(double minroll, double maxroll, double x_1, double y_1, double x_2,
              double y_2, double time_1, double time_2) {
    timer park_timer;
    double roll = Inertial.roll(rotationUnits::deg);
    std::cout << "Starting autoPark, initial roll: " << roll << std::endl;

    // 第一阶段：慢速前进直到检测到爬坡（roll > minroll）
    while (roll > minroll && park_timer.time() < time_1) {
        roll = Inertial.roll(rotationUnits::deg);
        myDrive.setAbsAuton(Vector(x_1, y_1), 0);
        this_thread::sleep_for(10);
    }
    std::cout << "phase1_angle: " << roll << ", time: " << park_timer.time() << "ms"
              << std::endl;

    // 第二阶段：继续前进直到到达顶部（roll < maxroll）
    while (roll < maxroll && park_timer.time() < time_2) {
        roll = Inertial.roll(rotationUnits::deg);
        myDrive.setAbsAuton(Vector(x_2, y_2), 0);
        this_thread::sleep_for(10);
    }
    std::cout << "phase2_angle: " << roll << ", time: " << park_timer.time() << "ms"
              << std::endl;

    // 停止
    myDrive.stop(brake);
    this_thread::sleep_for(400);

    // 输出最终角度
    double final_roll = Inertial.roll(rotationUnits::deg);
    std::cout << "autoPark completed! Final roll: " << final_roll << " degrees"
              << std::endl;
}

#endif
