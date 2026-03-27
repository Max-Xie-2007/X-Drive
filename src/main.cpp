#include "autonomous.h"
#include "basic/block-path.h"
#include "basic/robot-config.h"
#include "debug.h"
#include "driver-control.h"
#include "ui/design.h"
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
    thread TUpdateBlockPath(updateBlockPath);
    thread TDebug(debugControl);

    Competition.drivercontrol(driverControl);
    Competition.autonomous(autonomous);

    while (true) {
        this_thread::sleep_for(10);
    }
}
