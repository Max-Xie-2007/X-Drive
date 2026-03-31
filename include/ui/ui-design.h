#ifndef UI_DESIGN_H
#define UI_DESIGN_H

#include "ui/ui-base.h"

/**
 * @brief 清空控制器屏幕
 */
void clearController();
/**
 * @brief 清空主脑屏幕
 */
void clearBrain();
/**
 * @brief 显示顶部状态栏
 */
void displayTopBar();
/**
 * @brief 显示选项页
 */
void displayOptions();
/**
 * @brief 显示电机信息页
 */
void displayMotors();
/**
 * @brief 显示传感器信息页
 */
void displayDevices();
/**
 * @brief 显示常规信息页
 */
void displayInfo();
/**
 * @brief 显示参数调试页
 */
void displayParams();

/**
 * @brief 更新主脑 UI
 */
void updateBrainUI();
/**
 * @brief 更新控制器 UI
 */
void updateControllerUI();
/**
 * @brief 更新全部 UI
 */
void updateUI();

#endif // UI_DESIGN_H
