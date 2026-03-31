#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "chassis/driving.h"
#include "chassis/positioning.h"
#include "globals.h"
#include "vex.h"
#include <vector>

using namespace vex;
using namespace std;

extern brain Brain;
extern controller Controller;

extern inertial Inertial;
extern rotation Rotation_L;
extern rotation Rotation_R;

inline constexpr int base_motor_count = 1;
extern motor Motors_Base_LF[base_motor_count];
extern motor Motors_Base_LB[base_motor_count];
extern motor Motors_Base_RF[base_motor_count];
extern motor Motors_Base_RB[base_motor_count];

extern XDrive myDrive;

extern OrthogonalOdometry myPosition;
extern rotation Rotation_L;
extern rotation Rotation_R;

extern bool initializing;
void botInit();

#endif