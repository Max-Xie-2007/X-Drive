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
        reach(myPosition.getCenterPos() + Vector(0, 24), 0.0);
    }
    if (B) {
        reach(myPosition.getCenterPos() + Vector(0, -24), 0.0);
    }
    if (Y) {
        reach(myPosition.getCenterPos() + Vector(-24, 0), 0.0);
    }
    if (A) {
        reach(myPosition.getCenterPos() + Vector(24, 0), 0.0);
    }
    if (Left) {
        reach(myPosition.getCenterPos(), myPosition.getHeading() + 45.0);
    }
    if (Right) {
        reach(myPosition.getCenterPos(), myPosition.getHeading() - 45.0);
    }
    // if (L1) {
    //     reach(myPosition.getCenterPos(), myPosition.getHeading() + 90.0);
    // }
    // if (R1) {
    //     reach(myPosition.getCenterPos(), myPosition.getHeading() - 90.0);
    // }
    // if (R2) {
    //     reach(myPosition.getCenterPos(), myPosition.getHeading() + 180.0);
    // }
    // if (L2) {
    //     reach(myPosition.getCenterPos(), myPosition.getHeading() - 180.0);
    // }
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
