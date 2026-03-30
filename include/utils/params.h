#ifndef PARAMS_H
#define PARAMS_H

#include "utils/pid.h"
#include "vex.h"

struct Cycle {
    const short auton = 5;    // 自动刷新时间(ms)
    const short position = 5; // 定位刷新时间(ms)
    const short driver = 20;  // 遥控刷新时间(ms)
    const short ui = 100;     // ui刷新时间(ms)
};
extern const Cycle cycle;

struct Hardware {
    struct Odometer {
        const double pos_wheel_radius = 1.375;           // 定位轮半径(inch)
        const double IMU_heading_coef = 3600.0 / 3598.5; // 陀螺仪航向校正系数

        // 定位中心相对机器人中心的偏移(inch)
        // x轴正方向为机器人前方，y轴正方向为机器人左方
        const Vector offset = Vector(-2.6016, -0.1173);
        // 定位轮正方向：车沿该方向运动时编码器读数增加
        // 定位轮安装角度：车头沿着x轴正方向摆放时与x轴正方向的夹角，CCW-positive
        // 测量方式：将车沿x轴正方向移动一定距离，记录编码器读数，用反三角函数计算夹角
        const double left_encoder_angle = acos((82.98) / (24 * 5));
        const double right_encoder_angle = -acos((84.55) / (24 * 5));
        const double den = sin(right_encoder_angle - left_encoder_angle);
        // 定位轮所在直线交点（直接定位对象）关于车中心点的偏移，x正方向为车头
    } odometer;
};
extern const Hardware hardware;

extern PIDParam default_reach_translational;
extern PIDParam default_reach_angular;
extern PIDParam default_trace_translational;
extern PIDParam default_trace_angular;
extern PIDParam default_trace_back;

/**
 * @brief Reach配置
 * @param max_translational_speed 最大平移速度(%)
 * @param max_angular_speed 最大旋转速度(%)
 * @param if_hold 到达时是否保持制动
 * @param is_terminal 是否为终点路径段
 * @param translational_slew_rate 平移速度斜坡率(%/ms)
 * @param angular_slew_rate 旋转速度斜坡率(%/ms)
 */
struct ReachCfg {
    int max_translational_speed = 100;
    int max_angular_speed = 100;
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
    int max_angular_speed = 100;
    bool if_hold = true;
    bool is_terminal = true;
    float translational_slew_rate = 5.0;
    float angular_slew_rate = 5.0;
    double look_ahead_dist = 3.0;
};
extern TraceCfg default_trace_cfg;

#endif // PARAMS_H
