#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "basic/driving.h"
#include "basic/positioning.h"
#include "vex.h"
#include <vector>

using namespace vex;
using namespace std;

extern brain Brain;
extern controller Controller;

extern inertial Inertial;

extern optical Optical_Exit;
extern vex::distance Distance_Entrance;
extern vex::distance Distance_Exit;
extern motor Motor_Intaker1;
extern motor Motor_Intaker2;
extern motor_group Motors_Intaker;
extern motor Motor_Roller;
extern motor Motor_Shooter;

extern digital_out Piston_Load;
extern digital_out Piston_Shooter;
extern digital_out Piston_Hook;

#ifdef XDRIVE
extern motor Motor_Base_LFU;
extern motor Motor_Base_LFD;
extern motor Motor_Base_LBU;
extern motor Motor_Base_LBD;
extern motor Motor_Base_RBU;
extern motor Motor_Base_RBD;
extern motor Motor_Base_RFU;
extern motor Motor_Base_RFD;

extern motor_group Motors_Base_LF;
extern motor_group Motors_Base_LB;
extern motor_group Motors_Base_RF;
extern motor_group Motors_Base_RB;

extern XDrive myDrive;
#endif // XDRIVE

#ifdef STRAIGHT
extern motor Motor_Base_L1;
extern motor Motor_Base_L2;
extern motor Motor_Base_L3;
extern motor Motor_Base_L4;
extern motor Motor_Base_R1;
extern motor Motor_Base_R2;
extern motor Motor_Base_R3;
extern motor Motor_Base_R4;

extern motor_group Motors_Base_L;
extern motor_group Motors_Base_R;

extern Straight myDrive;
#endif // STRAIGHT

#ifdef POSITION_ORTHOGONAL
extern Orthogonal myPosition;
extern rotation Rotation_L;
extern rotation Rotation_R;
#endif // POSITION_ORTHOGONAL

#ifdef POSITION_DIFFERENTIAL
extern Differential myPosition;
extern rotation Rotation;
#endif // POSITION_DIFFERENTIAL

extern bool initializing;
void botInit();

#endif