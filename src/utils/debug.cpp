#include "utils/debug.h"
#include "auto-functions/motion.h"
#include "autonomous.h"
#include "chassis/positioning.h"
#include "controller.h"
#include "driver-control.h"
#include "params.h"
#include "robot-config.h"
#include "ui/ui-brain.h"
#include "ui/ui-controller.h"
#include "vex.h"
using namespace vex;

void squareCirc() {
    myPosition.reset(Point(0, 0));
    const Segment path1(Point(0, 0), Point(0, 12));
    const Arc path2 = Arc(Point(0, 12), Point(24, 36), 24, true, false);
    const Segment path3(Point(24, 36), Point(36, 36));
    const Arc path4(Point(36, 36), Point(60, 12), 24, true, false);
    const Segment path5(Point(60, 12), Point(60, 0));
    const Arc path6(Point(60, 0), Point(36, -24), 24, true, false);
    const Segment path7(Point(36, -24), Point(24, -24));
    const Arc path8(Point(24, -24), Point(0, 0), 24, true, false);
    traceWithGlobalHeading(path1, 0.0, 3000,
                           {.is_terminal = false, .min_translational_speed = 50},
                           default_trace_translational, default_trace_angular,
                           default_trace_deviational, {.exit_on_overshoot_ = true});
    traceWithGlobalHeading(path2, 0.0, 3000,
                           {.is_terminal = false, .min_translational_speed = 50},
                           default_trace_translational, default_trace_angular,
                           default_trace_deviational, {.exit_on_overshoot_ = true});
    traceWithGlobalHeading(path3, 0.0, 3000,
                           {.is_terminal = false, .min_translational_speed = 50},
                           default_trace_translational, default_trace_angular,
                           default_trace_deviational, {.exit_on_overshoot_ = true});
    traceWithGlobalHeading(path4, 0.0, 3000,
                           {.is_terminal = false, .min_translational_speed = 50},
                           default_trace_translational, default_trace_angular,
                           default_trace_deviational, {.exit_on_overshoot_ = true});
    traceWithGlobalHeading(path5, 0.0, 3000,
                           {.is_terminal = false, .min_translational_speed = 50},
                           default_trace_translational, default_trace_angular,
                           default_trace_deviational, {.exit_on_overshoot_ = true});
    traceWithGlobalHeading(path6, 0.0, 3000,
                           {.is_terminal = false, .min_translational_speed = 50},
                           default_trace_translational, default_trace_angular,
                           default_trace_deviational, {.exit_on_overshoot_ = true});
    traceWithGlobalHeading(path7, 0.0, 3000,
                           {.is_terminal = false, .min_translational_speed = 50},
                           default_trace_translational, default_trace_angular,
                           default_trace_deviational, {.exit_on_overshoot_ = true});
    traceWithGlobalHeading(path8, 0.0);
}

void pentagram() {
    myPosition.reset(Point(0, 0));
    const Segment path1(Point(0, 0), Point(8, 24));
    const Segment path2(Point(8, 24), Point(-16, 8));
    const Segment path3(Point(-16, 8), Point(16, 8));
    const Segment path4(Point(16, 8), Point(-8, 24));
    const Segment path5(Point(-8, 24), Point(0, 0));
    traceWithGlobalHeading(path1, 0.0);
    traceWithGlobalHeading(path2, 0.0);
    traceWithGlobalHeading(path3, 0.0);
    traceWithGlobalHeading(path4, 0.0);
    traceWithGlobalHeading(path5, 0.0);
}

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
        reach(myPosition.getCenterPos(), myPosition.getHeading() + 45.0, 2000,
              default_reach_cfg, default_reach_translational, default_reach_angular_45);
    }
    if (Right) {
        reach(myPosition.getCenterPos(), myPosition.getHeading() - 45.0, 2000,
              default_reach_cfg, default_reach_translational, default_reach_angular_45);
    }
    if (L1) {
        reach(myPosition.getCenterPos(), myPosition.getHeading() + 90.0, 2000,
              default_reach_cfg, default_reach_translational, default_reach_angular_90);
    }
    if (R1) {
        reach(myPosition.getCenterPos(), myPosition.getHeading() - 90.0, 2000,
              default_reach_cfg, default_reach_translational, default_reach_angular_90);
    }
    if (R2) {
        reach(myPosition.getCenterPos(), myPosition.getHeading() + 135.0, 2000,
              default_reach_cfg, default_reach_translational, default_reach_angular_135);
    }
    if (L2) {
        reach(myPosition.getCenterPos(), myPosition.getHeading() - 135.0, 2000,
              default_reach_cfg, default_reach_translational, default_reach_angular_135);
    }
    if (Down) {
        pentagram();
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
