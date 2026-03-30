#ifndef AUTO_FUNCTIONS_MOTION_H
#define AUTO_FUNCTIONS_MOTION_H

#include "utils/pid.h"
#include "robot-config.h"
#include "chassis/driving.h"
#include "chassis/positioning.h"
#include "vex.h"
#include <vector>

using namespace vex;

/********************** CHASSIS CONTROL **********************/

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
                              
#endif // AUTO_FUNCTIONS_MOTION_H