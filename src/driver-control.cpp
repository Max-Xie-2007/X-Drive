#include "driver-control.h"
#include "utils/params.h"
#include "robot-config.h"
#include "ui/ui-design.h"
#include "utils/calc.h"
#include "vex.h"
#include <iostream>

void driverControl() {
    myDrive.setBrakeType(coast);
    myDrive.setDriveMethod(VOLT);
    myDrive.setImportantFactor(NEITHER);
    while (true) {
        if (current_mode != PID_DEBUG) {
            switchRouteNColor();
        }
        chassisControl();
        this_thread::sleep_for(cycle.driver);
    }
}

/*********************** 键位表 ***********************/
// SWITCH ROUTE & COLOR
#define BTN_SWITCH_ROUTE (Controller.ButtonRight.pressing())
#define BTN_SWITCH_COLOR (Controller.ButtonLeft.pressing())
// CHASSIS
#define JOY_CHASSIS_LR (Controller.Axis4.position(pct))
#define JOY_CHASSIS_FB (Controller.Axis3.position(pct))
#define JOY_CHASSIS_TURN ((-Controller.Axis1.position(pct)))
// BLOCK PATH
#define BTN_INTAKE_IN (Controller.ButtonL1.pressing())
#define BTN_LCG (Controller.ButtonL2.pressing() && !Controller.ButtonL1.pressing())
#define BTN_LG (Controller.ButtonR1.pressing() && !Controller.ButtonR2.pressing())
#define BTN_UCG (Controller.ButtonR2.pressing() && !Controller.ButtonR1.pressing())
// PISTON
#define BTN_LOAD (Controller.ButtonL1.pressing() && Controller.ButtonL2.pressing())
#define BTN_HOOK (Controller.ButtonR1.pressing() && Controller.ButtonR2.pressing())
/******************************************************/

// 保护变量，一个trigger操作对应一个变量
bool bSwitchRoute = false;
bool bSwitchColor = false;
bool bHook = false;

void chassisControl() {
    int lr = deadZone(JOY_CHASSIS_LR, 3);
    int fb = deadZone(JOY_CHASSIS_FB, 3);
    int rot = deadZone(JOY_CHASSIS_TURN, 3);
    // 第一人称（第三人称用setAbsDriver)
    myDrive.setRelDriver(Vector(lr, fb), 0.8 * rot);
}

void switchRouteNColor() {
    if (BTN_SWITCH_ROUTE) {
        if (!bSwitchRoute) {
            int next_val = (static_cast<int>(current_route) + 1) % 4;
            current_route = static_cast<Route>(next_val);
            if (current_interface == OPTIONS) {
                displayOptions();
            }
            bSwitchRoute = true;
        }
    } else {
        bSwitchRoute = false;
    }
    if (BTN_SWITCH_COLOR) {
        if (!bSwitchColor) {
            int next_val = (static_cast<int>(current_color) + 1) % 2;
            current_color = static_cast<Color>(next_val);
            if (current_interface == OPTIONS) {
                displayOptions();
            }
            bSwitchColor = true;
        }
    } else {
        bSwitchColor = false;
    }
}
