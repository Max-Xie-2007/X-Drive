#include "robot-config.h"
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
