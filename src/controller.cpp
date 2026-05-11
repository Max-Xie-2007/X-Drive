#include "controller.h"
#include "driver-control.h"
#include "robot-config.h"
#include "vex.h"

int A1, A2, A3, A4, prev_A1, prev_A2, prev_A3, prev_A4;
bool L1, L2, R1, R2, X, Y, A, B, Left, Right, Up, Down;
bool prev_L1, prev_L2, prev_R1, prev_R2, prev_X, prev_Y, prev_A, prev_B, prev_Left,
    prev_Right, prev_Up, prev_Down;
void threadUpdateController() {
    while (true) {
        prev_A1 = A1;
        prev_A2 = A2;
        prev_A3 = A3;
        prev_A4 = A4;
        prev_L1 = L1;
        prev_L2 = L2;
        prev_R1 = R1;
        prev_R2 = R2;
        prev_X = X;
        prev_Y = Y;
        prev_A = A;
        prev_B = B;
        prev_Left = Left;
        prev_Right = Right;
        prev_Up = Up;
        prev_Down = Down;
        A1 = Controller.Axis1.position(vex::percentUnits::pct);
        A2 = Controller.Axis2.position(vex::percentUnits::pct);
        A3 = Controller.Axis3.position(vex::percentUnits::pct);
        A4 = Controller.Axis4.position(vex::percentUnits::pct);
        L1 = Controller.ButtonL1.pressing();
        L2 = Controller.ButtonL2.pressing();
        R1 = Controller.ButtonR1.pressing();
        R2 = Controller.ButtonR2.pressing();
        X = Controller.ButtonX.pressing();
        Y = Controller.ButtonY.pressing();
        A = Controller.ButtonA.pressing();
        B = Controller.ButtonB.pressing();
        Left = Controller.ButtonLeft.pressing();
        Right = Controller.ButtonRight.pressing();
        Up = Controller.ButtonUp.pressing();
        Down = Controller.ButtonDown.pressing();
        this_thread::sleep_for(cycle.driver_control);
    }
}

double getCurvedOutput(double input, ControllerCurveParams params) {
    if (fabs(input) <= params.dead_zone) {
        return 0;
    }
    const double dead_zone_input = input - sgn(input) * params.dead_zone;
    const double input_ratio =
        map(dead_zone_input, -100 + params.dead_zone, 100 - params.dead_zone, -1.0, 1.0);
    const double curved_ratio =
        sgn(input_ratio) * pow(fabs(input_ratio), params.exponent);
    const double curved_output = curved_ratio * 100.0;
    return curved_output;
}