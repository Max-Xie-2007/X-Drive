#include "chassis/driving.h"
#include "robot-config.h"
#include "utils/calc.h"
#include "vex.h"
#include <algorithm>
#include <cmath>

using namespace vex;

void threadUpdateDriving() {
    while (true) {
        myDrive.drive();
        this_thread::sleep_for(cycle.driving);
    }
}

void XDrive::setAbsAuton(Vector auton_trans_speed, double auton_rot_speed) {
    abs_auton_trans_ = auton_trans_speed;
    auton_ang_ = auton_rot_speed;
    rel_auton_trans_ = abs_auton_trans_.rotate(-myPosition.getHeading());
}
void XDrive::setAbsDriver(Vector driver_trans_speed, double driver_rot_speed) {
    abs_driver_trans_ = driver_trans_speed;
    driver_ang_ = driver_rot_speed;
    rel_driver_trans_ = abs_driver_trans_.rotate(-myPosition.getHeading());
}
void XDrive::setRelAuton(Vector auton_trans_speed, double auton_rot_speed) {
    rel_auton_trans_ = auton_trans_speed;
    auton_ang_ = auton_rot_speed;
    abs_auton_trans_ = rel_auton_trans_.rotate(myPosition.getHeading());
}
void XDrive::setRelDriver(Vector driver_trans_speed, double driver_rot_speed) {
    rel_driver_trans_ = driver_trans_speed;
    driver_ang_ = driver_rot_speed;
    abs_driver_trans_ = rel_driver_trans_.rotate(myPosition.getHeading());
}
void XDrive::setDriveMethod(DriveMethod method) { current_drive_method_ = method; }
void XDrive::setBrakeType(brakeType type) { current_brake_type_ = type; }
void XDrive::setPriorFactor(DriveFactor factor) { important_factor_ = factor; }

void XDrive::drive() {
    Vector translational_sum = rel_auton_trans_ + rel_driver_trans_;
    double angular_sum = auton_ang_ + driver_ang_;

    double trans_LF = translational_sum.projectOn(Vector(-1, 1));
    double trans_LB = translational_sum.projectOn(Vector(-1, -1));
    double trans_RF = translational_sum.projectOn(Vector(1, 1));
    double trans_RB = translational_sum.projectOn(Vector(1, -1));
    double max_trans =
        std::max({fabs(trans_LF), fabs(trans_LB), fabs(trans_RF), fabs(trans_RB)});

    double input_LF_raw = trans_LF + angular_sum;
    double input_LB_raw = trans_LB + angular_sum;
    double input_RF_raw = trans_RF + angular_sum;
    double input_RB_raw = trans_RB + angular_sum;
    double max_input_raw = std::max(
        {fabs(input_LF_raw), fabs(input_LB_raw), fabs(input_RF_raw), fabs(input_RB_raw)});

    switch (important_factor_) {
        case TRANS:
            if (max_input_raw > 100) {
                // Keep translation as-is, shrink rotation to the remaining headroom.
                const double rot_abs = std::fabs(angular_sum);
                if (rot_abs > 1e-6) {
                    const double rot_budget = std::max(0.0, 100.0 - max_trans);
                    const double scale = std::min(1.0, rot_budget / rot_abs);
                    angular_sum *= scale;
                }
            }
            break;
        case ANGULAR:
            if (max_input_raw > 100) {
                // Keep rotation as-is, shrink translation to the remaining headroom.
                if (max_trans > 1e-6) {
                    const double trans_budget =
                        std::max(0.0, 100.0 - std::fabs(angular_sum));
                    const double scale = std::min(1.0, trans_budget / max_trans);
                    translational_sum = translational_sum * scale;
                }
            }
            break;
        case NEITHER:
            break;
    }

    double input_LF = translational_sum.projectOn(Vector(-1, 1)) + angular_sum;
    double input_LB = translational_sum.projectOn(Vector(-1, -1)) + angular_sum;
    double input_RF = translational_sum.projectOn(Vector(1, 1)) + angular_sum;
    double input_RB = translational_sum.projectOn(Vector(1, -1)) + angular_sum;

    if (current_drive_method_ == VOLT) {
        for (int i = 0; i < motor_count_; i++) {
            if (fabs(input_LF) > 1) {
                motors_lf_[i].spin(fwd, pct2volt(input_LF), volt);
            } else {
                motors_lf_[i].stop(current_brake_type_);
            }
            if (fabs(input_LB) > 1) {
                motors_lb_[i].spin(fwd, pct2volt(input_LB), volt);
            } else {
                motors_lb_[i].stop(current_brake_type_);
            }
            if (fabs(input_RF) > 1) {
                motors_rf_[i].spin(fwd, pct2volt(input_RF), volt);
            } else {
                motors_rf_[i].stop(current_brake_type_);
            }
            if (fabs(input_RB) > 1) {
                motors_rb_[i].spin(fwd, pct2volt(input_RB), volt);
            } else {
                motors_rb_[i].stop(current_brake_type_);
            }
        }
    } else if (current_drive_method_ == PCT) {
        for (int i = 0; i < motor_count_; i++) {
            if (fabs(input_LF) > 1) {
                motors_lf_[i].spin(fwd, input_LF, pct);
            } else {
                motors_lf_[i].stop(current_brake_type_);
            }
            if (fabs(input_LB) > 1) {
                motors_lb_[i].spin(fwd, input_LB, pct);
            } else {
                motors_lb_[i].stop(current_brake_type_);
            }
            if (fabs(input_RF) > 1) {
                motors_rf_[i].spin(fwd, input_RF, pct);
            } else {
                motors_rf_[i].stop(current_brake_type_);
            }
            if (fabs(input_RB) > 1) {
                motors_rb_[i].spin(fwd, input_RB, pct);
            } else {
                motors_rb_[i].stop(current_brake_type_);
            }
        }
    }
}

void XDrive::stop(brakeType type) {
    setAbsAuton(Vector(0, 0), 0);
    setAbsDriver(Vector(0, 0), 0);
    current_brake_type_ = type;
    for (int i = 0; i < motor_count_; i++) {
        motors_lf_[i].stop(type);
        motors_lb_[i].stop(type);
        motors_rf_[i].stop(type);
        motors_rb_[i].stop(type);
    }
}