#include "utils/debug.h"
#include "auto-functions/motion.h"
#include "autonomous.h"
#include "utils/params.h"
#include "chassis/positioning.h"
#include "robot-config.h"
#include "ui/ui-base.h"
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
        reach(cur_pos + Vector(0, 24), cur_heading);
    }
    if (BTN_DEBUG_BWD) {
        Point cur_pos = myPosition.getCenterPos();
        double cur_heading = myPosition.getHeading();
        reach(cur_pos + Vector(0, -24), cur_heading);
    }
    if (BTN_DEBUG_ROTL) {
        Point cur_pos = myPosition.getCenterPos();
        double cur_heading = myPosition.getHeading();
        reach(cur_pos, cur_heading + 90);
    }
    if (BTN_DEBUG_ROTR) {
        Point cur_pos = myPosition.getCenterPos();
        double cur_heading = myPosition.getHeading();
        reach(cur_pos, cur_heading - 90);
    }
}
