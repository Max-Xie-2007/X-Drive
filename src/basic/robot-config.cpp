#include "basic/robot-config.h"
#include "ui/design.h"
using namespace vex;

brain Brain;
controller Controller;

#ifdef ROBOT_WHITE
inertial Inertial(PORT7, vex::left); // set to left for CCW-Positive
#ifdef POSITION_ORTHOGONAL
rotation Rotation_L = rotation(PORT1, true);
rotation Rotation_R = rotation(PORT9, false);
Orthogonal myPosition(Rotation_L, Rotation_R, Inertial);
#endif // POSITION_ORTHOGONAL

optical Optical_Exit = optical(PORT13);
vex::distance Distance_Entrance = vex::distance(PORT21);
vex::distance Distance_Exit = vex::distance(PORT20);

digital_out Piston_Load = digital_out(Brain.ThreeWirePort.H);
digital_out Piston_Shooter = digital_out(Brain.ThreeWirePort.E);
digital_out Piston_Hook = digital_out(Brain.ThreeWirePort.D);

motor Motor_Intaker1 = motor(PORT10, ratio36_1, false);
motor Motor_Intaker2 = motor(PORT11, ratio36_1, true);
motor_group Motors_Intaker = motor_group(Motor_Intaker1, Motor_Intaker2);
motor Motor_Roller = motor(PORT8, ratio36_1, true);
motor Motor_Shooter = motor(PORT16, ratio36_1, false);

#ifdef XDRIVE
motor Motor_Base_LFU = motor(PORT9, ratio6_1, false);
motor Motor_Base_LFD = motor(PORT7, ratio6_1, true);
motor Motor_Base_LBU = motor(PORT17, ratio6_1, false);
motor Motor_Base_LBD = motor(PORT10, ratio6_1, true);
motor Motor_Base_RBU = motor(PORT6, ratio6_1, false);
motor Motor_Base_RBD = motor(PORT8, ratio6_1, true);
motor Motor_Base_RFU = motor(PORT16, ratio6_1, false);
motor Motor_Base_RFD = motor(PORT14, ratio6_1, true);

motor_group Motors_Base_LF = motor_group(Motor_Base_LFU, Motor_Base_LFD);
motor_group Motors_Base_LB = motor_group(Motor_Base_LBU, Motor_Base_LBD);
motor_group Motors_Base_RF = motor_group(Motor_Base_RFU, Motor_Base_RFD);
motor_group Motors_Base_RB = motor_group(Motor_Base_RBU, Motor_Base_RBD);

XDrive myDrive(Motors_Base_LF, Motors_Base_LB, Motors_Base_RF, Motors_Base_RB);

#endif // XDRIVE

#ifdef STRAIGHT
motor Motor_Base_L1 = motor(PORT13, ratio6_1, true);
motor Motor_Base_L2 = motor(PORT14, ratio6_1, true);
motor Motor_Base_L3 = motor(PORT15, ratio6_1, false);
motor Motor_Base_L4 = motor(PORT2, ratio6_1, false); // empty
motor Motor_Base_R1 = motor(PORT18, ratio6_1, false);
motor Motor_Base_R2 = motor(PORT19, ratio6_1, false);
motor Motor_Base_R3 = motor(PORT20, ratio6_1, true);
motor Motor_Base_R4 = motor(PORT5, ratio6_1, true); // empty

motor_group Motors_Base_L =
    motor_group(Motor_Base_L1, Motor_Base_L2, Motor_Base_L3, Motor_Base_L4);
motor_group Motors_Base_R =
    motor_group(Motor_Base_R1, Motor_Base_R2, Motor_Base_R3, Motor_Base_R4);

Straight myDrive(Motors_Base_L, Motors_Base_R);
#endif // STRAIGHT
#endif // ROBOT_WHITE

#ifdef ROBOT_BLACK
inertial Inertial(PORT7, vex::left); // set to left for CCW-Positive
rotation Rotation = rotation(PORT1, true);

optical Optical_Exit = optical(PORT13);
vex::distance Distance_Entrance = vex::distance(PORT21);
vex::distance Distance_Exit = vex::distance(PORT20);

digital_out Piston_Load = digital_out(Brain.ThreeWirePort.H);
digital_out Piston_Shooter = digital_out(Brain.ThreeWirePort.E);
digital_out Piston_Hook = digital_out(Brain.ThreeWirePort.D);

motor Motor_Intaker1 = motor(PORT10, ratio36_1, false);
motor Motor_Intaker2 = motor(PORT11, ratio36_1, true);
motor_group Motors_Intaker = motor_group(Motor_Intaker1, Motor_Intaker2);
motor Motor_Roller = motor(PORT8, ratio36_1, true);
motor Motor_Shooter = motor(PORT16, ratio36_1, false);

#ifdef XDRIVE
motor Motor_Base_LFU = motor(PORT9, ratio6_1, false);
motor Motor_Base_LFD = motor(PORT7, ratio6_1, true);
motor Motor_Base_LBU = motor(PORT17, ratio6_1, false);
motor Motor_Base_LBD = motor(PORT10, ratio6_1, true);
motor Motor_Base_RBU = motor(PORT6, ratio6_1, false);
motor Motor_Base_RBD = motor(PORT8, ratio6_1, true);
motor Motor_Base_RFU = motor(PORT16, ratio6_1, false);
motor Motor_Base_RFD = motor(PORT14, ratio6_1, true);

motor_group Motors_Base_LF = motor_group(Motor_Base_LFU, Motor_Base_LFD);
motor_group Motors_Base_LB = motor_group(Motor_Base_LBU, Motor_Base_LBD);
motor_group Motors_Base_RF = motor_group(Motor_Base_RFU, Motor_Base_RFD);
motor_group Motors_Base_RB = motor_group(Motor_Base_RBU, Motor_Base_RBD);

XDrive myDrive(Motors_Base_LF, Motors_Base_LB, Motors_Base_RF, Motors_Base_RB);

#endif // XDRIVE

#ifdef STRAIGHT
motor Motor_Base_L1 = motor(PORT13, ratio6_1, true);
motor Motor_Base_L2 = motor(PORT14, ratio6_1, true);
motor Motor_Base_L3 = motor(PORT15, ratio6_1, false);
motor Motor_Base_L4 = motor(PORT2, ratio6_1, false); // empty
motor Motor_Base_R1 = motor(PORT18, ratio6_1, false);
motor Motor_Base_R2 = motor(PORT19, ratio6_1, false);
motor Motor_Base_R3 = motor(PORT20, ratio6_1, true);
motor Motor_Base_R4 = motor(PORT5, ratio6_1, true); // empty

motor_group Motors_Base_L =
    motor_group(Motor_Base_L1, Motor_Base_L2, Motor_Base_L3, Motor_Base_L4);
motor_group Motors_Base_R =
    motor_group(Motor_Base_R1, Motor_Base_R2, Motor_Base_R3, Motor_Base_R4);

Straight myDrive(Motors_Base_L, Motors_Base_R);
#endif // STRAIGHT

#ifdef POSITION_DIFFERENTIAL
Differential myPosition(Rotation, myDrive, Inertial);
#endif // POSITION_DIFFERENTIAL
#endif // ROBOT_BLACK

bool initializing;
void botInit() {
    initializing = true;
    this_thread::sleep_for(200);
    clearController();
    cout << "------------------Init------------------" << endl;

    // IMU Calibration
    timer calibration_timer;
    Controller.Screen.setCursor(3, 1);
    Controller.Screen.print("IMU Calibrating...");
    cout << "IMU Calibrating..." << endl;
    while (true) {
        Inertial.startCalibration();
        while (Inertial.isCalibrating()) {
            this_thread::sleep_for(10);
        }
        if (calibration_timer.time() < 1800) {
            Controller.Screen.setCursor(3, 1);
            Controller.Screen.print("Recalibrating...              ");
            cout << "Fake Calibration! Recalibrating..." << endl;
            calibration_timer.reset();
        } else {
            break;
        }
    }
    this_thread::sleep_for(200);
    double last_rot = Inertial.heading(deg);
    cout << "IMU Heading at 200ms: " << last_rot << endl;
    this_thread::sleep_for(800);
    if (abs(Inertial.heading(deg) - last_rot) > 0.1f) {
        Controller.rumble("..");
        Controller.Screen.setCursor(3, 1);
        Controller.Screen.print("IMU Error! Recal Suggested      ");

        cout << "Error! IMU Heading at 1000ms: " << Inertial.heading(deg) << endl;
    } else {
        Controller.rumble(".");
        Controller.Screen.setCursor(3, 1);
        Controller.Screen.print("IMU Ready!                      ");
        cout << "Ready! IMU Heading at 1000ms: " << Inertial.heading(deg) << endl;
    }
    myPosition.reset();
    this_thread::sleep_for(500);
    clearController();
    initializing = false;
}