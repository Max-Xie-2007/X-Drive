#include "autonomous.h"
#include "auto-functions/motion.h"
#include "robot-config.h"

void autonomous() {
    timer auton_timer;
    myDrive.setDriveMethod(VOLT);
    myDrive.setPriorFactor(ANGULAR);
    myDrive.setBrakeType(hold);
    switch (current_route) {
#ifdef LEFT
        case L1:
            autoL1();
            break;
        case L2:
            autoL2();
            break;
        case L3:
            autoL3();
            break;
#endif
#ifdef RIGHT
        case R1:
            autoR1();
            break;
        case R2:
            autoR2();
            break;
        case R3:
            autoR3();
            break;
#endif
        case SKILLS:
            autoSkills();
            break;
    }
    Controller.Screen.setCursor(3, 13);
    Controller.Screen.print("T: %.2fs", auton_timer.time() / 1000.0);
    std::cout << "Total Time: " << auton_timer.time() / 1000.0 << "s" << std::endl;
}