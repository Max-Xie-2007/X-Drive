#include "debug.h"
#include "auto-functions/motion.h"
#include "autonomous.h"
#include "basic/params.h"
#include "basic/positioning.h"
#include "basic/robot-config.h"
#include "ui/base.h"
#include "vex.h"
using namespace vex;

/*********************** 调试键位表 ***********************/
// RE-INIT & CALIBRATION
#define BTN_REINIT (Controller.ButtonUp.pressing())
// PID_DEBUG
#define BTN_DEBUG_FWD (Controller.ButtonX.pressing())
#define BTN_DEBUG_BWD (Controller.ButtonB.pressing())
#define BTN_DEBUG_ROTR (Controller.ButtonA.pressing())
#define BTN_DEBUG_ROTL (Controller.ButtonY.pressing())
#define BTN_DEBUG_POSEL (Controller.ButtonL1.pressing())
#define BTN_DEBUG_POSER (Controller.ButtonR1.pressing())
#define BTN_DEBUG_CONTINUOUS_POSE (Controller.ButtonDown.pressing())
// ROUTE_DEBUG
#define BTN_DEBUG_ROUTE (Controller.ButtonDown.pressing())
/*********************************************************/

void debugControl() {
    while (true) {
        if (current_mode != COMPETITION) {
            if (BTN_REINIT) {
                botInit();
            }
        }
        if (current_mode == PID_DEBUG) {
            chassisDebug();
        } else if (current_mode == ROUTE_DEBUG) {
            if (BTN_DEBUG_ROUTE) {
                autonomous();
            }
        }
        this_thread::sleep_for(cycle.driver);
    }
}

void chassisDebug() {
    if (BTN_DEBUG_FWD) {
        Point cur_pos = myPosition.getCenterPos();
        double cur_heading = myPosition.getHeading();
        reach(cur_pos + Vector(0, 24), cur_heading,
              Config{.reach = Config::Reach{.time_limit = 2000,
                                            .PID_trans = config.reach.PID_trans,
                                            .PID_rot = config.reach.PID_rot}});
    }
    if (BTN_DEBUG_BWD) {
        Point cur_pos = myPosition.getCenterPos();
        double cur_heading = myPosition.getHeading();
        reach(cur_pos + Vector(0, -24), cur_heading,
              Config{.reach = Config::Reach{.time_limit = 2000,
                                            .PID_trans = config.reach.PID_trans,
                                            .PID_rot = config.reach.PID_rot}});
    }
    if (BTN_DEBUG_ROTL) {
        Point cur_pos = myPosition.getCenterPos();
        double cur_heading = myPosition.getHeading();
        reach(cur_pos, cur_heading + 90,
              Config{.reach = Config::Reach{.time_limit = 2000,
                                            .PID_trans = config.reach.PID_trans,
                                            .PID_rot = config.reach.PID_rot}});
    }
    if (BTN_DEBUG_ROTR) {
        Point cur_pos = myPosition.getCenterPos();
        double cur_heading = myPosition.getHeading();
        reach(cur_pos, cur_heading - 90,
              Config{.reach = Config::Reach{.time_limit = 2000,
                                            .PID_trans = config.reach.PID_trans,
                                            .PID_rot = config.reach.PID_rot}});
    }
}
