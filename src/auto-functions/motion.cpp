#include "auto-functions/motion.h"
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
    Vector prev_translational_output = myPosition.getTranslationalVelocity();
    double prev_angular_error = degNorm180(target_heading - myPosition.getHeading());
    double prev_angular_output = myPosition.getAngularVelocity();
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
        const double angular_error = degNorm180(target_heading - myPosition.getHeading());
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
        // translational_output =
        //     desat(translational_output, reach_cfg.min_translational_speed);
        angular_output = sat(angular_output, reach_cfg.max_angular_speed);
        // angular_output = desat(angular_output, reach_cfg.min_angular_speed);

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
                            const PIDParam angular_pid, const PIDParam deviational_pid,
                            ExitConditionOptions translational_ec_opt,
                            ExitConditionOptions angular_ec_opt) {
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
    double prev_angular_error =
        degNorm180(target_heading - myPosition.getHeading());
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
    while (timeout_timer.time(msec) < time_limit &&
           !(translational_ec.reached() && angular_ec.reached())) {
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

        // update exit conditions
        translational_ec.update(translational_error, translational_dvt);
        angular_ec.update(angular_error, angular_dvt);

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

template <typename T>
void traceWithRelativeHeading(const T& path, double terminal_target_heading,
                              double target_heading_offset, int time_limit,
                              const TraceCfg& trace_cfg, const PIDParam translational_pid,
                              const PIDParam angular_pid, const PIDParam deviational_pid,
                              ExitConditionOptions translational_ec_opt,
                              ExitConditionOptions angular_ec_opt) {
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
    //

    // loop
    while (timeout_timer.time(msec) < time_limit &&
           !(translational_ec.reached() && angular_ec.reached())) {
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

        // update exit conditions
        translational_ec.update(translational_error, translational_dvt);
        angular_ec.update(angular_error, angular_dvt);

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