#include "utils/exit-condition.h"

ExitCondition::ExitCondition(double err_tol, double dvt_tol, int jump_time,
                             bool exit_on_overshoot)
    : err_tol_(err_tol), dvt_tol_(dvt_tol), jump_time_(jump_time),
      exit_on_overshoot_(exit_on_overshoot), init_err_sign_(0) {
    jump_timer_.reset();
}

ExitCondition::ExitCondition(const ExitConditionOptions& opt)
    : err_tol_(opt.err_tol), dvt_tol_(opt.dvt_tol), jump_time_(opt.jump_time),
      exit_on_overshoot_(opt.exit_on_overshoot), init_err_sign_(0) {
    jump_timer_.reset();
}

void ExitCondition::setExitOnOvershoot(bool enabled) { exit_on_overshoot_ = enabled; }

void ExitCondition::reset(double init_err) {
    init_err_sign_ = sgn(init_err);
    jump_timer_.reset();
}

void ExitCondition::update(double err, double dvt) {
    const int err_sign = sgn(err);
    if (exit_on_overshoot_ && init_err_sign_ && err_sign && err_sign != init_err_sign_) {
        reached_ = true;
        return;
    }
    if (std::fabs(err) <= err_tol_ && std::fabs(dvt) <= dvt_tol_) {
        reached_ = jump_timer_.time(vex::msec) >= jump_time_;
    } else {
        jump_timer_.reset();
        reached_ = false;
    }
}

ExitCondition default_translational_ec(0.5, 0.003, 50, false);
ExitCondition default_angular_ec(1.0, 0.003, 50, false);