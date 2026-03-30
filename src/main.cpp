#include "autonomous.h"
#include "robot-config.h"
#include "utils/debug.h"
#include "driver-control.h"
#include "ui/ui-design.h"
#include "vex.h"
#include <iostream>

using namespace vex;
using namespace std;

competition Competition;

int main() {
    cout << endl
         << endl
         << "=================== Program Start ===================" << endl
         << endl;
    if (current_mode == COMPETITION) {
        botInit();
    } else {
        cout << "Running in non-competition mode!" << endl << endl;
    }

    thread TUpdateUI(updateUI);
    thread TUpdateChassis(updateChassis);
    thread TUpdatePosition(updatePosition);
    thread TDebug(debugControl);

    Competition.drivercontrol(driverControl);
    Competition.autonomous(autonomous);

    while (true) {
        this_thread::sleep_for(10);
    }
}
