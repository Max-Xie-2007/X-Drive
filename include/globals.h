#ifndef GLOBALS_H
#define GLOBALS_H

#include <string>

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
/**
 * @brief 自动路线枚举（左场）
 */
enum Route { L1, L2, L3, SKILLS };
#endif
#ifdef RIGHT
/**
 * @brief 自动路线枚举（右场）
 */
enum Route { R1, R2, R3, SKILLS };
#endif
extern std::string auto_route_name[4];

/**
 * @brief 联盟颜色
 */
enum Color { RED, BLUE };
/**
 * @brief 运行模式
 */
enum Mode { COMPETITION, PID_DEBUG, ROUTE_DEBUG };

extern Mode current_mode;
extern Route current_route;
extern Color current_color;

extern bool expansion_complete;

/**
 * @brief 切换运行模式
 * @param new_mode 目标模式
 */
void changeModeTo(Mode new_mode);
/**
 * @brief 切换自动路线
 * @param new_route 目标路线
 */
void changeRouteTo(Route new_route);
/**
 * @brief 切换红蓝方
 */
void changeColor();

/**
 * @brief 轮换自动程序路线
 */
void switchRoute();

#endif // GLOBALS_H