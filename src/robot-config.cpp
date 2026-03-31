#include "robot-config.h"
#include "ui/ui-design.h"
using namespace vex;

brain Brain;
controller Controller;

inertial Inertial(PORT7, vex::left); // set to left for CCW-Positive
rotation Rotation_L = rotation(PORT1, true);
rotation Rotation_R = rotation(PORT2, true);

OrthogonalOdometry myPosition(Rotation_L, Rotation_R, Inertial);

motor Motors_Base_LF[base_motor_count] = {motor(PORT9, ratio6_1, false)};
motor Motors_Base_LB[base_motor_count] = {motor(PORT17, ratio6_1, false)};
motor Motors_Base_RB[base_motor_count] = {motor(PORT6, ratio6_1, false)};
motor Motors_Base_RF[base_motor_count] = {motor(PORT16, ratio6_1, false)};

XDrive myDrive(Motors_Base_LF, Motors_Base_LB, Motors_Base_RF, Motors_Base_RB,
               base_motor_count);

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