#ifndef DRIVER_CONTROL_H
#define DRIVER_CONTROL_H

#include "basic/driving.h"
void driverControl();

void blockPathControl();

void pistonControl();

/**
 * @brief 基于控制器输入驱动底盘（遥控模式）
 */
void chassisControl();

/**
 * @brief 切换自动程序路线和红蓝方
 */
void switchRouteNColor();

#endif // DRIVER_CONTROL_H