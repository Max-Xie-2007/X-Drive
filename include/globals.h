#ifndef GLOBALS_H
#define GLOBALS_H
#include "ui/ui-brain.h"
#include <iostream>

/************* ROBOT MACROS DEFINE *************/

#define ROBOT_GRAY
// #define ROBOT_PURPLE

/***********************************************/

enum class Driver { CHIFENG, CKS, ZYX, XTR, COUNT };
enum class Mode { COMPETITION, PRACTICE, PID_DEBUG, ROUTE_DEBUG, COUNT };
enum class Side { LEFT, RIGHT };
enum class Route { TOUR1, TOUR2, TOUR3, TOURD, SKILLS_COOP, SKILLS_SOLO, COUNT };
enum class Color { RED, BLUE, NEITHER, COUNT };
/**
 * @brief 机器人当前阶段枚举
 * @param PRE_CALIBRATION 校准前阶段，需完成机器折叠和
 * @param CALIBRATING 校准阶段，折叠完成后正在校准传感器的阶段
 * @param CALIBRATED 校准完成阶段，正在等待比赛开始的阶段
 * @param AUTON 自动阶段，正在执行自动程序的阶段
 * @param DRIVER_CONTROL 遥控阶段，正在执行遥控程序的阶段
 * @param COUNT 枚举数量
 */
enum class Stage {
    PRE_CALIBRATION,
    CALIBRATING,
    CALIBRATED,
    AUTON,
    DRIVER_CONTROL,
    COUNT
};

extern Driver current_driver;
extern Mode current_mode;
extern Side current_side;
extern Route current_route;
extern Color current_color;
extern Stage current_stage;
extern bool expansion_done;

void switchDriver();
void switchMode();
void switchSide();
void switchRoute();
void switchColor();

void setDriver(Driver driver);
void setMode(Mode mode);
void setSide(Side side);
void setRoute(Route route);
void setColor(Color color);

#endif // GLOBALS_H