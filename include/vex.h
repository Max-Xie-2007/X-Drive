#ifndef VEX_H
#define VEX_H

#include <cmath>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"

#define waitUntil(condition)                                                             \
    do {                                                                                 \
        wait(5, msec);                                                                   \
    } while (!(condition))
#define waitWhile(condition)                                                             \
    do {                                                                                 \
        wait(5, msec);                                                                   \
    } while (condition);

#define repeat(iterations) for (int iterator = 0; iterator < iterations; iterator++)

/************* ROBOT DEFINITION *************/

#define ROBOT_WHITE
// #define ROBOT_BLACK

/********************************************/

#ifdef ROBOT_WHITE
// #define STRAIGHT
#define XDRIVE
// #define POSITION_DIFFERENTIAL
#define POSITION_ORTHOGONAL
#define LEFT
#endif // ROBOT_WHITE

#ifdef ROBOT_BLACK
// #define XDRIVE
#define STRAIGHT
#define POSITION_DIFFERENTIAL
// #define POSITION_ORTHOGONAL
#define RIGHT
#endif // ROBOT_BLACK

#ifdef LEFT
enum Route { L1, L2, L3, SKILLS };
#endif
#ifdef RIGHT
enum Route { R1, R2, R3, SKILLS };
#endif
extern std::string auto_route_name[4];

enum Color { RED, BLUE };
enum Mode { COMPETITION, PID_DEBUG, ROUTE_DEBUG };
enum PathEvent { STOP, INTAKE, LCG, UCG, LG, EXPANSION };

extern Mode current_mode;
extern Route current_route;
extern Color current_color;

#endif // VEX_H