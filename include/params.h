#ifndef PARAMS_H
#define PARAMS_H

#include "utils/exit-condition.h"
#include "utils/pid.h"
#include <cmath>

struct Cycle {
    const short auton;                // 自动刷新时间(ms)
    const short driving;              // 驱动刷新时间(ms)
    const short position;             // 定位刷新时间(ms)
    const short driver_control;       // 遥控刷新时间(ms)
    const short brain_ui_update;      // 主机ui刷新时间(ms)
    const short controller_ui_update; // 手柄ui刷新时间(ms)
};
extern const Cycle cycle;

struct Hardware {
    /**
     * @param rotation_dist_per_deg 定位轮每转一度行进的距离(inch)
     * @param IMU_heading_coef 陀螺仪转一度对应的真实角度(deg)
     * @param IMU_damping_th 陀螺仪消抖阈值
     * @param left_encoder_angle 左编码器相对于机器人前方的安装角度(0..360, CCW-positive)
     * @param right_encoder_angle 右编码器相对于机器人前方的安装角度(0..360, CCW-positive)
     * @param den 坐标系变换矩阵行列式的值
     * @param offset
     * 定位中心相对机器人中心的偏移(inch)，x轴正方向为机器人前方，y轴正方向为机器人左方(非必要)
     */
    struct Odometer {
        double rotation_dist_per_deg;
        double IMU_heading_coef;
        double IMU_damping_th;
        double left_encoder_angle;
        double right_encoder_angle;
        double den = sin(right_encoder_angle - left_encoder_angle);
        Vector offset;
    } odometer;
};
extern const Hardware hardware;

/**
 * @brief Reach配置
 * @param max_translational_speed 最大平移速度(%)
 * @param min_translational_speed 最小平移速度(%)
 * @param max_angular_speed 最大旋转速度(%)
 * @param min_angular_speed 最小旋转速度(%)
 * @param if_hold 到达时是否保持制动
 * @param is_terminal 是否为终点路径段
 * @param translational_slew_rate 平移速度斜坡率(%/ms)
 * @param angular_slew_rate 旋转速度斜坡率(%/ms)
 */
struct ReachCfg {
    int max_translational_speed = 100;
    int min_translational_speed = 0;
    int max_angular_speed = 100;
    int min_angular_speed = 0;
    bool if_hold = true;
    bool is_terminal = true;
    float translational_slew_rate = 5.0;
    float angular_slew_rate = 5.0;
};
extern ReachCfg default_reach_cfg;

/**
 * @brief Trace配置
 * @param max_translational_speed 最大平移速度(%)
 * @param max_angular_speed 最大旋转速度(%)
 * @param if_hold 到达时是否保持制动
 * @param is_terminal 是否为终点路径段
 * @param translational_slew_rate 平移速度斜坡率(%/ms)
 * @param angular_slew_rate 旋转速度斜坡率(%/ms)
 * @param look_ahead_dist 前瞻距离(inch)
 */
struct TraceCfg {
    int max_translational_speed = 100;
    int min_translational_speed = 0;
    int max_angular_speed = 100;
    bool if_hold = true;
    bool is_terminal = true;
    float translational_slew_rate = 5.0;
    float angular_slew_rate = 5.0;
    double look_ahead_dist = 5.0;
};
extern TraceCfg default_trace_cfg;

extern PIDParam default_reach_translational;
extern PIDParam default_reach_angular_45;
extern PIDParam default_reach_angular_90;
extern PIDParam default_reach_angular_135;
extern PIDParam default_trace_translational;
extern PIDParam default_trace_angular;
extern PIDParam default_trace_deviational;

extern ExitConditionOptions default_translational_ec_opt;
extern ExitConditionOptions default_angular_ec_opt;

#endif // PARAMS_H
