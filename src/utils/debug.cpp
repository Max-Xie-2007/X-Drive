#include "utils/debug.h"
#include "auto-functions/motion.h"
#include "autonomous.h"
#include "chassis/positioning.h"
#include "controller.h"
#include "driver-control.h"
#include "robot-config.h"
#include "ui/ui-brain.h"
#include "ui/ui-controller.h"
#include "utils/params.h"
#include "vex.h"
using namespace vex;

void chassisDebug() {
    if (X) {
    }
    if (B) {
    }
    if (Y) {
    }
    if (A) {
    }
    if (Right) {
    }
    if (Left) {
    }
    if (L1) {
    }
    if (R1) {
    }
    if (R2) {
    }
    if (L2) {
    }
    if (Down) {
    }
}

void calibration4Debug() {
    if (Up && !prev_Up) {
        timer hold_timer;
        hold_timer.reset();
        bool long_press = false;
        while (Up) {
            if (hold_timer.time() >= 150) {
                long_press = true;
                break;
            }
            this_thread::sleep_for(cycle.driver_control);
        }
        if (long_press) {
            current_stage = Stage::CALIBRATING;
            calibrate();
            current_stage = Stage::CALIBRATED;
        } else {
            myPosition.reset(Point(0, 0), myPosition.getUnwrappedHeading());
        }
    }
}
