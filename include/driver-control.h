#ifndef DRIVER_CONTROL_H
#define DRIVER_CONTROL_H

#include "chassis/driving.h"

/**
 * @brief 遥控阶段主循环
 */
void driverControl();

/**
 * @brief 挡路机构控制
 */
void blockPathControl();

/**
 * @brief 气动装置控制
 */
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