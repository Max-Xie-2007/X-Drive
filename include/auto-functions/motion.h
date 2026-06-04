#ifndef AUTO_FUNCTIONS_MOTION_H
#define AUTO_FUNCTIONS_MOTION_H

#include "chassis/driving.h"
#include "params.h"
#include "robot-config.h"
#include "utils/exit-condition.h"
#include "utils/geometry.h"
#include "utils/pid.h"
#include "vex.h"

using namespace vex;

/********************** CHASSIS CONTROL **********************/

/**
 * @brief 移动机器人到目标位置并调整朝向
 * @param target_pos 目标位置（全局坐标系）
 * @param target_heading 目标朝向（度，CCW-positive）
 * @param time_limit 时间限制（ms）
 * @param reach_cfg 动作配置
 * @param translational_pid 平移PID参数
 * @param angular_pid 旋转PID参数
 * @param translational_ec 平移退出条件
 * @param angular_ec 旋转退出条件
 */
void reach(const Point& target_pos, double target_heading, int time_limit = 2000,
           const ReachCfg& reach_cfg = default_reach_cfg,
           const PIDParam translational_pid = default_reach_translational,
           const PIDParam angular_pid = default_reach_angular_90,
           ExitConditionOptions translational_ec = default_translational_ec_opt,
           ExitConditionOptions angular_ec = default_angular_ec_opt);

/**
 * @brief 跟踪路径并保持全局固定朝向
 * @param path 路径（线段）
 * @param target_heading 目标朝向（度，CCW-positive）
 * @param time_limit 时间限制（ms）
 * @param trace_cfg 跟踪配置
 * @param translational_pid 平移PID参数
 * @param angular_pid 旋转PID参数
 * @param deviational_pid 偏移PID参数
 * @param translational_ec 平移退出条件
 * @param angular_ec 旋转退出条件
 */
void traceWithGlobalHeading(
    const Segment& path, double target_heading, int time_limit = 2000,
    const TraceCfg& trace_cfg = default_trace_cfg,
    const PIDParam translational_pid = default_trace_translational,
    const PIDParam angular_pid = default_trace_angular,
    const PIDParam deviational_pid = default_trace_deviational,
    ExitConditionOptions translational_ec_opt = default_translational_ec_opt,
    ExitConditionOptions angular_ec_opt = default_angular_ec_opt);

/**
 * @brief 跟踪路径并保持全局固定朝向
 * @param path 路径（圆弧）
 * @param target_heading 目标朝向（度，CCW-positive）
 * @param time_limit 时间限制（ms）
 * @param trace_cfg 跟踪配置
 * @param translational_pid 平移PID参数
 * @param angular_pid 旋转PID参数
 * @param deviational_pid 偏移PID参数
 * @param translational_ec 平移退出条件
 * @param angular_ec 旋转退出条件
 */
void traceWithGlobalHeading(
    const Arc& path, double target_heading, int time_limit = 2000,
    const TraceCfg& trace_cfg = default_trace_cfg,
    const PIDParam translational_pid = default_trace_translational,
    const PIDParam angular_pid = default_trace_angular,
    const PIDParam deviational_pid = default_trace_deviational,
    ExitConditionOptions translational_ec_opt = default_translational_ec_opt,
    ExitConditionOptions angular_ec_opt = default_angular_ec_opt);

/**
 * @brief 跟踪路径并保持相对朝向（相对于机器人速度方向）
 * @param path 路径段（线段）
 * @param terminal_target_heading 最终目标朝向（度，CCW-positive），!is_terminal时有效
 * @param target_heading_offset 朝向偏移（度，CCW-positive），相对于机器人速度方向的偏移
 * @param time_limit 时间限制（ms）
 * @param trace_cfg 动作配置
 * @param translational_pid 平移PID参数
 * @param angular_pid 旋转PID参数
 * @param return_pid 反向修正PID参数
 * @param translational_ec 平移退出条件
 * @param angular_ec 旋转退出条件
 */
void traceWithRelativeHeading(
    const Segment& path, double terminal_target_heading, double target_heading_offset = 0.0,
    int time_limit = 2000, const TraceCfg& trace_cfg = default_trace_cfg,
    const PIDParam translational_pid = default_trace_translational,
    const PIDParam angular_pid = default_trace_angular,
    const PIDParam deviational_pid = default_trace_deviational,
    ExitConditionOptions translational_ec_opt = default_translational_ec_opt,
    ExitConditionOptions angular_ec_opt = default_angular_ec_opt);

/**
 * @brief 跟踪路径并保持相对朝向（相对于机器人速度方向）
 * @param path 路径段（圆弧）
 * @param terminal_target_heading 最终目标朝向（度，CCW-positive），!is_terminal时有效
 * @param target_heading_offset 朝向偏移（度，CCW-positive），相对于机器人速度方向的偏移
 * @param time_limit 时间限制（ms）
 * @param trace_cfg 动作配置
 * @param translational_pid 平移PID参数
 * @param angular_pid 旋转PID参数
 * @param return_pid 反向修正PID参数
 * @param translational_ec 平移退出条件
 * @param angular_ec 旋转退出条件
 */
void traceWithRelativeHeading(
    const Arc& path, double terminal_target_heading, double target_heading_offset = 0.0,
    int time_limit = 2000, const TraceCfg& trace_cfg = default_trace_cfg,
    const PIDParam translational_pid = default_trace_translational,
    const PIDParam angular_pid = default_trace_angular,
    const PIDParam deviational_pid = default_trace_deviational,
    ExitConditionOptions translational_ec_opt = default_translational_ec_opt,
    ExitConditionOptions angular_ec_opt = default_angular_ec_opt);
#endif // AUTO_FUNCTIONS_MOTION_H