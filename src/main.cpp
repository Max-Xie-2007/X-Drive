#include "autonomous.h"
#include "controller.h"
#include "driver-control.h"
#include "globals.h"
#include "robot-config.h"

#include "ui/ui-brain.h"
#include "ui/ui-controller.h"
#include "utils/debug.h"
#include "vex.h"

#include <iostream>
using namespace vex;
using namespace std;

competition myCompetition;

int main() {
    // competition setup
    myCompetition.drivercontrol(driverControl);
    myCompetition.autonomous(autonomous);
    wait(200, msec);
    current_stage = Stage::PRE_CALIBRATION;

    thread TUpdateController(threadUpdateController);
    thread TUpdateBrainUI(threadUpdateBrainUI);
    thread TUpdateControllerUI(threadUpdateControllerUI);
    thread TUpdateDriving(threadUpdateDriving);
    thread TUpdatePosition(threadUpdatePosition);

    // log
    cout << endl
         << "======================================================" << endl
         << "=================== Program Starts ===================" << endl
         << "======================================================" << endl
         << endl;

    // exit prevention
    while (true)
        wait(10, msec);

    return 0;
}
