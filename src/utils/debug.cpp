#include "utils/debug.h"
#include "auto-functions/motion.h"
#include "autonomous.h"
#include "chassis/positioning.h"
#include "controller.h"
#include "driver-control.h"
#include "params.h"
#include "robot-config.h"
#include "subsystem/painter.h"
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

const Point p1(0, 0);
const Point p2(5.85, 18);
const Point p3(-9.45, 6.85);
const Point p4(9.45, 6.85);
const Point p5(-5.85, 18);

void pentagram() {
    startPainting();
    const Segment path1(p1, p2);
    const Segment path2(p2, p3);
    const Segment path3(p3, p4);
    const Segment path4(p4, p5);
    const Segment path5(p5, p1);
    traceWithGlobalHeading(path1, 0.0, 3000, {.max_translational_speed = 20});
    traceWithGlobalHeading(path2, 0.0, 3000, {.max_translational_speed = 20});
    traceWithGlobalHeading(path3, 0.0, 3000, {.max_translational_speed = 20});
    traceWithGlobalHeading(path4, 0.0, 3000, {.max_translational_speed = 20});
    traceWithGlobalHeading(path5, 0.0, 3000, {.max_translational_speed = 20});
    stopPainting();
}

void pentagon() {
    startPainting();
    const Segment path1(p1, p4);
    const Segment path2(p4, p2);
    const Segment path3(p2, p5);
    const Segment path4(p5, p3);
    const Segment path5(p3, p1);
    traceWithGlobalHeading(path1, 0.0, 3000, {.max_translational_speed = 20});
    traceWithGlobalHeading(path2, 0.0, 3000, {.max_translational_speed = 20});
    traceWithGlobalHeading(path3, 0.0, 3000, {.max_translational_speed = 20});
    traceWithGlobalHeading(path4, 0.0, 3000, {.max_translational_speed = 20});
    traceWithGlobalHeading(path5, 0.0, 3000, {.max_translational_speed = 20});
    stopPainting();
}

void circle() {
    startPainting();
    const Arc path1(p1, p4, 9.93, false, false);
    const Arc path2(p4, p2, 9.93, false, false);
    const Arc path3(p2, p5, 9.93, false, false);
    const Arc path4(p5, p3, 9.93, false, false);
    const Arc path5(p3, p1, 9.93, false, false);
    traceWithGlobalHeading(path1, 0.0, 3000,
                           {.max_translational_speed = 20, .look_ahead_dist = 2},
                           {.kP_ = 40.0}, default_trace_angular,
                           {.kP_ = 20.0}, {.err_tol_ = 0.5});
    traceWithGlobalHeading(path2, 0.0, 3000,
                           {.max_translational_speed = 20, .look_ahead_dist = 2},
                           {.kP_ = 40.0}, default_trace_angular,
                           {.kP_ = 20.0}, {.err_tol_ = 0.5});
    traceWithGlobalHeading(path3, 0.0, 3000,
                           {.max_translational_speed = 20, .look_ahead_dist = 2},
                           {.kP_ = 40.0}, default_trace_angular,
                           {.kP_ = 20.0}, {.err_tol_ = 0.5});
    traceWithGlobalHeading(path4, 0.0, 3000,
                           {.max_translational_speed = 20, .look_ahead_dist = 2},
                           {.kP_ = 40.0}, default_trace_angular,
                           {.kP_ = 20.0}, {.err_tol_ = 0.5});
    traceWithGlobalHeading(path5, 0.0, 3000,
                           {.max_translational_speed = 20, .look_ahead_dist = 2},
                           {.kP_ = 40.0}, default_trace_angular,
                           {.kP_ = 20.0}, {.err_tol_ = 0.5});
    stopPainting();
}

void leftArrow() {
    startPainting();
    const float scalar = 2.0;
    const Segment path1(Point(0, 0), Point(3 * scalar, 0));
    const Segment path2(Point(3 * scalar, 0), Point(3 * scalar, 10 * scalar));
    const Arc path3(Point(3 * scalar, 10 * scalar), Point(-2 * scalar, 15 * scalar),
                    5 * scalar, false, false);
    const Segment path4(Point(-2 * scalar, 15 * scalar), Point(-4 * scalar, 15 * scalar));
    const Segment path5(Point(-4 * scalar, 15 * scalar), Point(-4 * scalar, 18 * scalar));
    const Segment path6(Point(-4 * scalar, 18 * scalar),
                        Point(-7 * scalar, 13.5 * scalar));
    const Segment path7(Point(-7 * scalar, 13.5 * scalar),
                        Point(-4 * scalar, 9 * scalar));
    const Segment path8(Point(-4 * scalar, 9 * scalar), Point(-4 * scalar, 12 * scalar));
    const Segment path9(Point(-4 * scalar, 12 * scalar), Point(-2 * scalar, 12 * scalar));
    const Arc path10(Point(-2 * scalar, 12 * scalar), Point(0, 10 * scalar), 2 * scalar,
                     true, false);
    const Segment path11(Point(0, 10 * scalar), Point(0, 0));
    traceWithGlobalHeading(path1, 0.0, 3000, {.max_translational_speed = 20});
    traceWithGlobalHeading(path2, 0.0, 3000, {.max_translational_speed = 20});
    traceWithGlobalHeading(path3, 0.0, 3000,
                           {.max_translational_speed = 20, .look_ahead_dist = 2},
                           {.kP_ = 40.0}, default_trace_angular,
                           {.kP_ = 20.0}, {.err_tol_ = 0.5});
    traceWithGlobalHeading(path4, 0.0, 3000, {.max_translational_speed = 20});
    traceWithGlobalHeading(path5, 0.0, 3000, {.max_translational_speed = 20});
    traceWithGlobalHeading(path6, 0.0, 3000, {.max_translational_speed = 20});
    traceWithGlobalHeading(path7, 0.0, 3000, {.max_translational_speed = 20});
    traceWithGlobalHeading(path8, 0.0, 3000, {.max_translational_speed = 20});
    traceWithGlobalHeading(path9, 0.0, 3000, {.max_translational_speed = 20});
    traceWithGlobalHeading(path10, 0.0, 3000,
                           {.is_terminal = false, .max_translational_speed = 20, .look_ahead_dist = 2},
                           {.kP_ = 40.0}, default_trace_angular,
                           default_trace_deviational, {.err_tol_ = 0.5, .exit_on_overshoot_ = true});
    traceWithGlobalHeading(path11, 0.0, 4000, {.max_translational_speed = 20});
    stopPainting();
}

void chassisDebug() {
    if (X) {
        pentagon();
    }
    if (B) {
        pentagram();
    }
    if (Y) {
        circle();
    }
    if (A) {
    }
    if (Left) {
        leftArrow();
    }
    if (Right) {
    }
    if (L1) {
    }
    if (L2) {
    }
    if (R1) {
    }
    if (R2) {
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
