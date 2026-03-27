#include "basic/driving.h"
#include "basic/robot-config.h"
#include "utils/calc.h"
#include "vex.h"
#include <algorithm>
#include <cmath>

using namespace vex;

void updateChassis() {
    while (true) {
        myDrive.drive();
        this_thread::sleep_for(cycle.driver);
    }
}

#ifdef XDRIVE

XDrive::XDrive(motor_group& motors_lf, motor_group& motors_lb, motor_group& motors_rf,
               motor_group& motors_rb)
    : motors_lf_(motors_lf), motors_lb_(motors_lb), motors_rf_(motors_rf),
      motors_rb_(motors_rb) {}

void XDrive::setAbsAuton(Vector auton_trans_speed, double auton_rot_speed) {
    abs_auton_trans_ = auton_trans_speed;
    auton_rot_speed_ = auton_rot_speed;
    rel_auton_trans_ = abs_auton_trans_.rotate(myPosition.getHeading());
}
void XDrive::setAbsDriver(Vector driver_trans_speed, double driver_rot_speed) {
    abs_driver_trans_ = driver_trans_speed;
    driver_rot_speed_ = driver_rot_speed;
    rel_driver_trans_ = abs_driver_trans_.rotate(myPosition.getHeading());
}
void XDrive::setRelAuton(Vector auton_trans_speed, double auton_rot_speed) {
    rel_auton_trans_ = auton_trans_speed;
    auton_rot_speed_ = auton_rot_speed;
    abs_auton_trans_ = abs_auton_trans_.rotate(-myPosition.getHeading());
}
void XDrive::setRelDriver(Vector driver_trans_speed, double driver_rot_speed) {
    rel_driver_trans_ = driver_trans_speed;
    driver_rot_speed_ = driver_rot_speed;
    abs_driver_trans_ = abs_driver_trans_.rotate(-myPosition.getHeading());
}
void XDrive::setDriveMethod(DriveMethod method) { current_drive_method_ = method; }
void XDrive::setBrakeType(brakeType type) { current_brake_type_ = type; }
void XDrive::setImportantFactor(DriveFactor factor) { important_factor_ = factor; }

void XDrive::drive() {
    Vector trans_sum = rel_auton_trans_ + rel_driver_trans_;
    double rot_sum = auton_rot_speed_ + driver_rot_speed_;

    double trans_LF = trans_sum.projectOn(Vector(1, 1));
    double trans_LB = trans_sum.projectOn(Vector(-1, 1));
    double trans_RF = trans_sum.projectOn(Vector(-1, 1));
    double trans_RB = trans_sum.projectOn(Vector(1, 1));
    double max_trans =
        std::max({fabs(trans_LF), fabs(trans_LB), fabs(trans_RF), fabs(trans_RB)});

    double input_LF_raw = trans_LF - rot_sum;
    double input_LB_raw = trans_LB - rot_sum;
    double input_RF_raw = trans_RF + rot_sum;
    double input_RB_raw = trans_RB + rot_sum;
    double max_input_raw = std::max(
        {fabs(input_LF_raw), fabs(input_LB_raw), fabs(input_RF_raw), fabs(input_RB_raw)});

    switch (important_factor_) {
        case TRANS:
            if (max_input_raw > 100) {
                // Keep translation as-is, shrink rotation to the remaining headroom.
                const double rot_abs = std::fabs(rot_sum);
                if (rot_abs > 1e-6) {
                    const double rot_budget = std::max(0.0, 100.0 - max_trans);
                    const double scale = std::min(1.0, rot_budget / rot_abs);
                    rot_sum *= scale;
                }
            }
            break;
        case ROT:
            if (max_input_raw > 100) {
                // Keep rotation as-is, shrink translation to the remaining headroom.
                if (max_trans > 1e-6) {
                    const double trans_budget = std::max(0.0, 100.0 - std::fabs(rot_sum));
                    const double scale = std::min(1.0, trans_budget / max_trans);
                    trans_sum = trans_sum * scale;
                }
            }
            break;
        case NEITHER:
            break;
    }

    double input_LF = trans_sum.projectOn(Vector(1, 1)) - rot_sum;
    double input_LB = trans_sum.projectOn(Vector(-1, 1)) - rot_sum;
    double input_RF = trans_sum.projectOn(Vector(-1, 1)) + rot_sum;
    double input_RB = trans_sum.projectOn(Vector(1, 1)) + rot_sum;

    if (current_drive_method_ == VOLT) {
        motors_lf_.spin(fwd, pct2volt(input_LF), volt);
        motors_lb_.spin(fwd, pct2volt(input_LB), volt);
        motors_rf_.spin(fwd, pct2volt(input_RF), volt);
        motors_rb_.spin(fwd, pct2volt(input_RB), volt);
    } else if (current_drive_method_ == PCT) {
        motors_lf_.spin(fwd, input_LF, pct);
        motors_lb_.spin(fwd, input_LB, pct);
        motors_rf_.spin(fwd, input_RF, pct);
        motors_rb_.spin(fwd, input_RB, pct);
    }
}

void XDrive::stop(brakeType type) {
    setAbsAuton(Vector(0, 0), 0);
    motors_lf_.stop(type);
    motors_lb_.stop(type);
    motors_rf_.stop(type);
    motors_rb_.stop(type);
}

#endif

#ifdef STRAIGHT

Straight::Straight(motor_group& motors_left, motor_group& motors_right)
    : motors_left_(motors_left), motors_right_(motors_right) {}

void Straight::setAuton(double fb, double rot) {
    auton_fb_speed_ = fb;
    auton_rot_speed_ = rot;
}
void Straight::setDriver(double fb, double rot) {
    driver_fb_speed_ = fb;
    driver_rot_speed_ = rot;
}
void Straight::setDriveMethod(DriveMethod method) { current_drive_method_ = method; }
void Straight::setBrakeType(brakeType type) { current_brake_type_ = type; }
void Straight::setImportantFactor(DriveFactor factor) { important_factor_ = factor; }

void Straight::resetPosL() { motors_left_.resetPosition(); }
void Straight::resetPosR() { motors_right_.resetPosition(); }
void Straight::resetPos() {
    resetPosL();
    resetPosR();
}

void Straight::drive() {
    double fb_sum = auton_fb_speed_ + driver_fb_speed_;
    double rot_sum = auton_rot_speed_ + driver_rot_speed_;
    switch (important_factor_) {
        case TRANS:
            rot_sum = sat(rot_sum, 100 - fabs(fb_sum));
            break;
        case ROT:
            fb_sum = sat(fb_sum, 100 - fabs(rot_sum));
            break;
        case NEITHER:
            break;
    }
    double input_L = fb_sum - rot_sum;
    double input_R = fb_sum + rot_sum;
    if (current_drive_method_ == VOLT) {
        if (input_L)
            motors_left_.spin(fwd, pct2volt(input_L), volt);
        else
            motors_left_.stop(current_brake_type_);
        if (input_R)
            motors_right_.spin(fwd, pct2volt(input_R), volt);
        else
            motors_right_.stop(current_brake_type_);
    } else if (current_drive_method_ == PCT) {
        if (input_L)
            motors_left_.spin(fwd, input_L, pct);
        else
            motors_left_.stop(current_brake_type_);
        if (input_R)
            motors_right_.spin(fwd, input_R, pct);
        else
            motors_right_.stop(current_brake_type_);
    }
}
void Straight::stop(brakeType type) {
    setAuton(0, 0);
    motors_left_.stop(type);
    motors_right_.stop(type);
}
#endif
