#include "globals.h"

// Driver current_driver = Driver::CHIFENG;
Driver current_driver = Driver::XTR;
// Driver current_driver = Driver::CKS;
// Driver current_driver = Driver::ZYX;

// Mode current_mode = Mode::COMPETITION;
// Mode current_mode = Mode::PRACTICE;
// Mode current_mode = Mode::ROUTE_DEBUG;
Mode current_mode = Mode::PID_DEBUG;

// Side current_side = Side::LEFT;
Side current_side = Side::RIGHT;

Route current_route = Route::TOUR1;
// Route current_route = Route::TOUR2;
// Route current_route = Route::TOUR3;
// Route current_route = Route::TOURD;
// Route current_route = Route::SKILLS_COOP;
// Route current_route = Route::SKILLS_SOLO;

Color current_color = Color::RED;
// Color current_color = Color::BLUE;
// Color current_color = Color::NEITHER;

Stage current_stage = Stage::PRE_CALIBRATION;
#ifdef ROBOT_GRAY
bool expansion_done = true;
#elif defined(ROBOT_PURPLE)
bool expansion_done = current_mode != Mode::COMPETITION && current_mode != Mode::PRACTICE;
#endif

void switchSide() {
    current_side = current_side == Side::LEFT ? Side::RIGHT : Side::LEFT;
    if (current_interface == Interface::AUTON) {
        displayAuton();
    }
}
void switchDriver() {
    current_driver = static_cast<Driver>((static_cast<int>(current_driver) + 1) %
                                         static_cast<int>(Driver::COUNT));
    if (current_interface == Interface::OPTIONS) {
        displayOptions();
    }
}
void switchMode() {
    current_mode = static_cast<Mode>((static_cast<int>(current_mode) + 1) %
                                     static_cast<int>(Mode::COUNT));
    if (current_interface == Interface::OPTIONS) {
        displayOptions();
    }
}
void switchRoute() {
    current_route = static_cast<Route>((static_cast<int>(current_route) + 1) %
                                       static_cast<int>(Route::COUNT));
    if (current_interface == Interface::AUTON) {
        displayAuton();
    }
}
void switchColor() {
    current_color = static_cast<Color>((static_cast<int>(current_color) + 1) %
                                       static_cast<int>(Color::COUNT));
    if (current_interface == Interface::OPTIONS) {
        displayOptions();
    }
}

void setSide(Side side) {
    current_side = side;
    if (current_interface == Interface::AUTON) {
        displayAuton();
    }
}
void setDriver(Driver driver) {
    current_driver = driver;
    if (current_interface == Interface::OPTIONS) {
        displayOptions();
    }
}
void setMode(Mode mode) {
    current_mode = mode;
    if (current_interface == Interface::OPTIONS) {
        displayOptions();
    }
}
void setRoute(Route route) {
    current_route = route;
    if (current_interface == Interface::AUTON) {
        displayAuton();
    }
}
void setColor(Color color) {
    current_color = color;
    if (current_interface == Interface::OPTIONS) {
        displayOptions();
    }
}