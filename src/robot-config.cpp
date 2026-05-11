#include "robot-config.h"
using namespace vex;

brain Brain;
controller Controller;

inertial Inertial(PORT15, vex::left);
rotation Rotation_L = rotation(PORT14, false);
rotation Rotation_R = rotation(PORT6, true);

OrthogonalOdometry myPosition(Rotation_L, Rotation_R, Inertial);

motor Motors_Base_LF[base_motor_count] = {motor(PORT11, ratio6_1, false)};
motor Motors_Base_LB[base_motor_count] = {motor(PORT12, ratio6_1, false)};
motor Motors_Base_RF[base_motor_count] = {motor(PORT21, ratio6_1, false)};
motor Motors_Base_RB[base_motor_count] = {motor(PORT10, ratio6_1, false)};

XDrive myDrive(Motors_Base_LF, Motors_Base_LB, Motors_Base_RF, Motors_Base_RB,
               base_motor_count);
