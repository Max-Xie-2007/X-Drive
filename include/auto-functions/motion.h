#ifndef AUTO_FUNCTIONS_MOTION_H
#define AUTO_FUNCTIONS_MOTION_H

#include "utils/pid.h"
#include "basic/robot-config.h"
#include "basic/driving.h"
#include "basic/positioning.h"
#include "vex.h"
#include <vector>

using namespace vex;

/********************** CHASSIS CONTROL **********************/

#ifdef XDRIVE
/**
 * @brief 移动机器人到目标位置并调整朝向
 * @param target_pos 目标位置（全局坐标系）
 * @param target_heading 目标朝向（度，CCW-positive）
 * @param cfg 动作配置
 */
void reach(const Point& target_pos, double target_heading, const Config& cfg = config);

/**
 * @brief 跟踪路径并保持全局固定朝向
 * @param path 路径段（线段）
 * @param target_heading 目标朝向（度，CCW-positive）
 * @param cfg 动作配置
 */
template <typename T>
void traceWithGlobalHeading(const T& path, double target_heading,
                            const Config& cfg = config);

/**
 * @brief 跟踪路径并保持相对朝向（相对于机器人速度方向）
 * @param path 路径段
 * @param heading_offset 相对角度差（度，CCW-positive）
 * @param terminal_target_heading 终点目标朝向（仅 is_terminal=true 时生效）
 * @param cfg 动作配置
 */
template <typename T>
void traceWithRelativeHeading(const T& path, double heading_offset = 0,
                              double terminal_target_heading = 0,
                              const Config& cfg = config);
#endif

#ifdef STRAIGHT

/**
 * @brief 控制底盘前进到指定距离并维持目标朝向
 * @param target_distance 目标前进距离（inch，前进为正、后退为负）
 * @param target_heading 目标朝向（度，CCW-positive）
 * @param cfg 动作配置
 * @return 实际前进的编码器平均角度（deg）
 */
double pidForward(double target_distance, double target_heading,
                  const Config& cfg = config);

/**
 * @brief 原地旋转到底盘目标朝向，并同步抑制左右轮差
 * @param target_heading 目标朝向（度，CCW-positive）
 * @param cfg 动作配置
 */
void turnTo(double target_heading, const Config& cfg = config);
/**
 * @brief 应用carrot point算法到达指定位置和朝向
 * @param target_pos 目标位置（全局坐标系）
 * @param target_heading 目标朝向（度，CCW-positive），指的是车头朝向
 * @param reversed 是否倒车进点
 * @param cfg 动作配置
 */
void moveToPose(const Point& target_pos, double target_heading, bool reversed,
                const Config& cfg = config);

/**
 * @brief 跟踪线段路径并保持相对朝向（相对于机器人速度方向）
 * @param path 路径段
 * @param final_heading 终点目标朝向
 * @param cfg 动作配置
 */
template <typename T>
void trace(const T& path, double final_heading = 0, const Config& cfg = config);

#endif

/** @brief 自动停车
 * @param minroll 最小roll角度阈值（开始检测到爬坡）
 * @param maxroll 最大roll角度阈值（到达停车台顶部）
 * @param x_1 第一阶段前进的x速度分量（正数）
 * @param y_1 第一阶段前进的y速度分量（负数）
 * @param x_2 第二阶段前进的x速度分量（正数）
 * @param y_2 第二阶段前进的y速度分量（负数）
 */
void autoPark(double minroll, double maxroll, double x_1, double y_1, double x_2,
              double y_2, double time_1, double time_2);

#endif // AUTO_FUNCTIONS_MOTION_H