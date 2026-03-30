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

extern motor Motor_Base_LF;
extern motor Motor_Base_LB;
extern motor Motor_Base_RF;
extern motor Motor_Base_RB;

extern XDrive myDrive;

extern Orthogonal myPosition;
extern rotation Rotation_L;
extern rotation Rotation_R;

extern bool initializing;
void botInit();

#endif