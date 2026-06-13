#include "driver-control.h"
#include "autonomous.h"
#include "controller.h"
#include "globals.h"
#include "params.h"
#include "robot-config.h"
#include "subsystem/painter.h"
#include "ui/ui-brain.h"
#include "ui/ui-controller.h"
#include "utils/calc.h"
#include "utils/debug.h"
#include "vex.h"

void driverControl() {
    current_stage = Stage::DRIVER_CONTROL;
    myDrive.setBrakeType(coast);
    myDrive.setAbsAuton(0, 0);
    myDrive.setPriorFactor(NEITHER);
    while (true) {
        if (current_mode == Mode::COMPETITION) {
            switch (current_stage) {
                case Stage::PRE_CALIBRATION:
                    preFoldingInit();
                    switchAutonControl();
                    switchColorControl();
                    if (!prev_A && A) {
                        current_stage = Stage::CALIBRATING;
                        calibrate();
                        current_stage = Stage::CALIBRATED;
                    }
                    break;
                case Stage::CALIBRATING:
                    break;
                case Stage::CALIBRATED:
                    myDrive.setBrakeType(coast);
                    myDrive.setPriorFactor(NEITHER);
                    chassisControl();
                    switchColorControl();
                    break;
                case Stage::AUTON:
                    break;
                case Stage::DRIVER_CONTROL:
                    myDrive.setBrakeType(coast);
                    myDrive.setPriorFactor(NEITHER);
                    chassisControl();
                    switchColorControl();
                    pistonControl();
                    break;
                default:
                    break;
            }
        } else if (current_mode == Mode::PRACTICE) {
            switch (current_stage) {
                case Stage::PRE_CALIBRATION:
                    preFoldingInit();
                    current_stage = Stage::DRIVER_CONTROL;
                    break;
                case Stage::CALIBRATING:
                    break;
                case Stage::CALIBRATED:
                    current_stage = Stage::DRIVER_CONTROL;
                    break;
                case Stage::AUTON:
                    break;
                case Stage::DRIVER_CONTROL:
                    myDrive.setBrakeType(coast);
                    myDrive.setPriorFactor(NEITHER);
                    chassisControl();
                    switchColorControl();
                    pistonControl();
                    switchAutonControl();
                    break;
                default:
                    break;
            }
        } else if (current_mode == Mode::ROUTE_DEBUG) {
            switch (current_stage) {
                case Stage::PRE_CALIBRATION:
                    preFoldingInit();
                    current_stage = Stage::DRIVER_CONTROL;
                    break;
                case Stage::CALIBRATING:
                    break;
                case Stage::CALIBRATED:
                    current_stage = Stage::DRIVER_CONTROL;
                    break;
                case Stage::AUTON:
                    break;
                case Stage::DRIVER_CONTROL:
                    myDrive.setBrakeType(coast);
                    myDrive.setPriorFactor(NEITHER);
                    chassisControl();
                    switchAutonControl();
                    switchColorControl();
                    calibration4Debug();
                    pistonControl();
                    if (!prev_Down && Down) {
                        autonomous();
                        current_stage = Stage::DRIVER_CONTROL;
                    }
                    break;
                default:
                    break;
            }
        } else if (current_mode == Mode::PID_DEBUG) {
            switch (current_stage) {
                case Stage::PRE_CALIBRATION:
                    preFoldingInit();
                    current_stage = Stage::DRIVER_CONTROL;
                    break;
                case Stage::CALIBRATING:
                    break;
                case Stage::CALIBRATED:
                    current_stage = Stage::DRIVER_CONTROL;
                    break;
                case Stage::AUTON:
                    break;
                case Stage::DRIVER_CONTROL:
                    myDrive.setBrakeType(coast);
                    myDrive.setPriorFactor(NEITHER);
                    chassisControl();
                    chassisDebug();
                    calibration4Debug();
                    pistonControl();
                    break;
                default:
                    break;
            }
        }
        this_thread::sleep_for(cycle.driver_control);
    }
}

void chassisControl() {
    const int lateral = getCurvedOutput(A4, {});
    const int forward = getCurvedOutput(A3, {});
    const int angular =
        getCurvedOutput(-A1, {.exponent = current_driver == Driver::CKS ? 1.0 : 1.012});
    myDrive.setAbsDriver(Vector(lateral, forward), angular); // 第三人称
    // myDrive.setRelDriver(Vector(lateral, forward).rotate(-90), angular); // 第一人称
}

void pistonControl() {
    if (L1) {
        stopPainting();
    } else if (L2) {
        startPainting();
    }
}

void switchAutonControl() {
    if (Left && !prev_Left) {
        timer hold_timer;
        hold_timer.reset();
        bool long_press = false;
        while (Left) {
            if (hold_timer.time() >= 250) {
                long_press = true;
                break;
            }
            this_thread::sleep_for(cycle.driver_control);
        }
        if (long_press) {
            switchSide();
        } else {
            switchRoute();
        }
    }
}
void switchColorControl() {
    if (current_driver == Driver::CHIFENG || current_driver == Driver::XTR) {
        if (Right && !prev_Right) {
            switchColor();
        }
    } else if (current_driver == Driver::CKS || current_driver == Driver::ZYX) {
        if (Up && !prev_Up) {
            switchColor();
        }
    }
}

void preFoldingInit() { stopPainting(); }

void calibrate() {
    timer calibration_timer;
    if (Inertial.installed()) {
        while (true) {
            calibration_timer.reset();
            while (true) {
                Inertial.startCalibration();
                while (Inertial.isCalibrating()) {
                    this_thread::sleep_for(cycle.auton);
                }
                if (calibration_timer.time() < 1800) {
                    calibration_timer.reset();
                } else {
                    break;
                }
            }
            this_thread::sleep_for(200);
            const double prev_heading = Inertial.heading(deg);
            this_thread::sleep_for(800);
            if (fabs(Inertial.heading(deg) - prev_heading) < 0.1) {
                Controller.rumble(".");
                break;
            }
        }
    } else {
        Controller.rumble("...");
    }
    myPosition.reset(Point(0, 0), 0);
}