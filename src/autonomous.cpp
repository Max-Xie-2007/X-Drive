#include "autonomous.h"
#include "auto-functions/motion.h"
#include "chassis/positioning.h"
#include "globals.h"
#include "robot-config.h"

#include <iomanip>
void autonomous() {
    current_stage = Stage::AUTON;
    timer auton_timer;
    switch (current_route) {
        case Route::TOUR1:
            current_side == Side::LEFT ? autoL1() : autoR1();
            break;
        case Route::TOUR2:
            current_side == Side::LEFT ? autoL2() : autoR2();
            break;
        case Route::TOUR3:
            current_side == Side::LEFT ? autoL3() : autoR3();
            break;
        case Route::TOURD:
            current_side == Side::LEFT ? autoLD() : autoRD();
            break;
        case Route::SKILLS_COOP:
            current_side == Side::LEFT ? autoLSkillsCoop() : autoRSkillsCoop();
            break;
        case Route::SKILLS_SOLO:
            autoSkillsSolo();
            break;
        default:
            break;
    }
    std::cout << std::fixed << std::setprecision(2)
              << "Total Time: " << auton_timer.time() / 1000.0 << "s" << std::endl;
    if (current_mode == Mode::COMPETITION) {

    } else {
        myDrive.stop(coast);
    }
}
void autoL1() {}
void autoL2() {}
void autoL3() {}
void autoLD() {}
void autoR1() {}
void autoR2() {}
void autoR3() {}
void autoRD() {}
void autoLSkillsCoop() {}
void autoRSkillsCoop() {}
void autoSkillsSolo() {}