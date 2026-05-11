#include "utils/exit-condition.h"
ExitCondition1D::ExitCondition1D(const ExitConditionOptions& opt)
    : ExitConditionOptions(opt) {
    jump_timer_.reset();
}

void ExitCondition1D::reset(double init_err) {
    prev_err_sign_ = sgn(init_err);
    jump_timer_.reset();
}

void ExitCondition1D::update(double err, double dvt) {
    const int err_sign = sgn(err);
    if (exit_on_overshoot_ && err_sign * prev_err_sign_ < 0) {
        reached_ = true;
        return;
    }
    prev_err_sign_ = err_sign;
    if (std::fabs(err) <= err_tol_ && std::fabs(dvt) <= dvt_tol_) {
        reached_ = jump_timer_.time(vex::msec) >= jump_time_;
    } else {
        jump_timer_.reset();
        reached_ = false;
    }
}

ExitCondition2D::ExitCondition2D(const ExitConditionOptions& opt)
    : ExitConditionOptions(opt), prev_err_vec_(0, 0) {
    jump_timer_.reset();
}

void ExitCondition2D::reset(Vector init_err_vec) {
    prev_err_vec_ = init_err_vec;
    jump_timer_.reset();
}

void ExitCondition2D::update(Vector err_vec, Vector dvt_vec) {
    if (exit_on_overshoot_ && prev_err_vec_.dot(err_vec) < 0) {
        reached_ = true;
        return;
    }
    prev_err_vec_ = err_vec;
    if (err_vec.len() <= err_tol_ && dvt_vec.len() <= dvt_tol_) {
        reached_ = jump_timer_.time(vex::msec) >= jump_time_;
    } else {
        jump_timer_.reset();
        reached_ = false;
    }
}
