#include "chassis/driving.h"
#include "robot-config.h"
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

XDrive::XDrive(motor& motor_lf, motor& motor_lb, motor& motor_rf, motor& motor_rb)
    : motor_lf_(motor_lf), motor_lb_(motor_lb), motor_rf_(motor_rf), motor_rb_(motor_rb) {
}

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
void XDrive::setPriorFactor(DriveFactor factor) { important_factor_ = factor; }

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
        case ANGULAR:
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
        motor_lf_.spin(fwd, pct2volt(input_LF), volt);
        motor_lb_.spin(fwd, pct2volt(input_LB), volt);
        motor_rf_.spin(fwd, pct2volt(input_RF), volt);
        motor_rb_.spin(fwd, pct2volt(input_RB), volt);
    } else if (current_drive_method_ == PCT) {
        motor_lf_.spin(fwd, input_LF, pct);
        motor_lb_.spin(fwd, input_LB, pct);
        motor_rf_.spin(fwd, input_RF, pct);
        motor_rb_.spin(fwd, input_RB, pct);
    }
}

void XDrive::stop(brakeType type) {
    setAbsAuton(Vector(0, 0), 0);
    motor_lf_.stop(type);
    motor_lb_.stop(type);
    motor_rf_.stop(type);
    motor_rb_.stop(type);
}