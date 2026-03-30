#ifndef GLOBALS_H
#define GLOBALS_H
#include "vex.h"

/************* ROBOT DEFINITION *************/

// #define ROBOT_WHITE
#define ROBOT_BLACK

/********************************************/

#ifdef ROBOT_WHITE
#define LEFT
#endif // ROBOT_WHITE

#ifdef ROBOT_BLACK
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

extern Mode current_mode;
extern Route current_route;
extern Color current_color;

extern bool expansion_complete;

void changeModeTo(Mode new_mode);
void changeRouteTo(Route new_route);
void changeColor();

void switchRoute();

#endif // GLOBALS_H