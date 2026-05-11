#include "ui/ui-controller.h"
#include "globals.h"
#include "robot-config.h"

#include "utils/pid.h"

void clearController() { Controller.Screen.clearScreen(); }

void printConfirmationPage() {
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("[A]=Confirm&Calibrate      ");
    Controller.Screen.setCursor(2, 1);
    switch (current_route) {
        case Route::SKILLS_COOP:
            Controller.Screen.print("Expected: %d deg           ",
                                    current_side == Side::LEFT ? 180 : 180);
            break;
        case Route::SKILLS_SOLO:
            Controller.Screen.print("Expected: %d deg           ", -90);
            break;
        default:
            Controller.Screen.print("Expected: %d deg           ",
                                    current_side == Side::LEFT ? 180 : 0);
            break;
    }
}
void printCalibrationPage() {
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("                           ");
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("Calibrating...             ");
}
void printSensorValues() {
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("L %.1f R %.1f            ", myPosition.getLeftRotDist(),
                            myPosition.getRightRotDist());
}
void printPosition() {
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("X %.1f Y %.1f H %.2f       ", myPosition.getCenterPos().x_,
                            myPosition.getCenterPos().y_,
                            myPosition.getUnwrappedHeading());
}
void printRouteNColor() {
    Controller.Screen.setCursor(3, 1);
    switch (current_color) {
        case Color::RED:
            Controller.Screen.print("RED      ");
            break;
        case Color::BLUE:
            Controller.Screen.print("BLUE     ");
            break;
        case Color::NEITHER:
            Controller.Screen.print("NEITHER  ");
            break;
        default:
            break;
    }
    Controller.Screen.setCursor(3, 10);
    switch (current_route) {
        case Route::TOUR1:
            Controller.Screen.print(current_side == Side::LEFT ? "L TOUR1             "
                                                               : "R TOUR1             ");
            break;
        case Route::TOUR2:
            Controller.Screen.print(current_side == Side::LEFT ? "L TOUR2             "
                                                               : "R TOUR2             ");
            break;
        case Route::TOUR3:
            Controller.Screen.print(current_side == Side::LEFT ? "L TOUR3             "
                                                               : "R TOUR3             ");
            break;
        case Route::TOURD:
            Controller.Screen.print(current_side == Side::LEFT ? "L TOURD             "
                                                               : "R TOURD             ");
            break;
        case Route::SKILLS_COOP:
            Controller.Screen.print(current_side == Side::LEFT
                                        ? "L SKL_COOP            "
                                        : "R SKL_COOP            ");
            break;
        case Route::SKILLS_SOLO:
            Controller.Screen.print("SKL_SOLO            ");
            break;
        default:
            break;
    }
}
void threadUpdateControllerUI() {
    clearController();
    while (true) {
        switch (current_stage) {
            case Stage::PRE_CALIBRATION:
                printConfirmationPage();
                printRouteNColor();
                break;
            case Stage::CALIBRATING:
                printCalibrationPage();
                printRouteNColor();
                break;
            case Stage::CALIBRATED:
            case Stage::AUTON:
            case Stage::DRIVER_CONTROL:
                printSensorValues();
                printPosition();
                printRouteNColor();
                break;
            default:
                break;
        }
        this_thread::sleep_for(cycle.controller_ui_update);
    }
}