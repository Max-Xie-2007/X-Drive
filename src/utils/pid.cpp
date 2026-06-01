#include "utils/pid.h"
#include "params.h"
#include "ui/ui-brain.h"
#include "ui/ui-controller.h"
#include "utils/calc.h"
PID1D::PID1D(const PIDParam& param) : PIDParam(param) {}
void PID1D::setParam(const PIDParam& param) {
    kP_ = param.kP_;
    kI_ = param.kI_;
    kD_ = param.kD_;
    I_th_ = param.I_th_;
    I_max_ = param.I_max_;
    sign_flip_clear_ = param.sign_flip_clear_;
}
void PID1D::update(double err_crt) {
    if (first_update_) {
        track_timer_.reset();
        err_int_ = 0;
        err_prev_ = err_crt;
        dt_ = 0;
        first_update_ = false;
    } else {
        dt_ = (track_timer_.time() - prev_time_);
    }
    err_crt_ = err_crt;
    err_int_ += err_crt_ * dt_ / 1000.0;
    // 若未足够接近目标或产生过冲则清零积分项
    if ((sign_flip_clear_ && err_prev_ * err_crt_ < 0) || fabs(err_crt_) > I_th_) {
        err_int_ = 0;
    }
    err_dvt_ = (dt_ ? (err_crt_ - err_prev_) / dt_ * 1000 : 0);

    P_ = err_crt_ * kP_;
    I_ = sat(err_int_ * kI_, I_max_);
    D_ = err_dvt_ * kD_;
    output_ = P_ + I_ + D_;

    prev_time_ = track_timer_.time();
    err_prev_ = err_crt_;
}
void PID1D::reset() {
    err_crt_ = 0;
    err_prev_ = 0;
    err_int_ = 0;
    err_dvt_ = 0;
    first_update_ = true;
    track_timer_.reset();
}

PID2D::PID2D(const PIDParam& param) : PIDParam(param) {}
void PID2D::setParam(const PIDParam& param) {
    kP_ = param.kP_;
    kI_ = param.kI_;
    kD_ = param.kD_;
    I_th_ = param.I_th_;
    I_max_ = param.I_max_;
    sign_flip_clear_ = param.sign_flip_clear_;
}
void PID2D::setCoeff(double kP, double kI, double kD) {
    kP_ = kP;
    kI_ = kI;
    kD_ = kD;
}

void PID2D::update(const Vector& err_crt) {
    if (first_update_) {
        err_int_ = Vector(0, 0);
        err_prev_ = err_crt;
        dt_ = 0;
        first_update_ = false;
        track_timer_.reset();
    } else {
        dt_ = (track_timer_.time() - prev_time_);
    }
    err_crt_ = err_crt;
    err_int_ = err_int_ + err_crt_ * dt_ / 1000.0;
    // 若未足够接近目标或产生过冲(积分向量与误差向量夹角为钝角)则清零积分向量项
    if ((sign_flip_clear_ && err_prev_.dot(err_crt_) < 0) || err_crt_.len() > I_th_) {
        err_int_ = 0;
    }
    err_dvt_ = (dt_ ? (err_crt_ - err_prev_) / dt_ * 1000 : Vector(0, 0));

    P_ = err_crt_ * kP_;
    I_ = sat(err_int_ * kI_, I_max_);
    D_ = err_dvt_ * kD_;
    output_ = P_ + I_ + D_;

    prev_time_ = track_timer_.time();
    err_prev_ = err_crt_;
}
void PID2D::reset() {
    err_crt_ = 0;
    err_prev_ = 0;
    err_int_ = 0;
    err_dvt_ = 0;
    first_update_ = true;
    track_timer_.reset();
}
