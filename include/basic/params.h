#ifndef PARAMS_H
#define PARAMS_H

#include "utils/pid.h"
#include "vex.h"

struct Cycle {
    const short auton = 10;   // 自动刷新时间(ms)
    const short position = 5; // 定位刷新时间(ms)
    const short driver = 20;  // 遥控刷新时间(ms)
    const short ui = 100;     // ui刷新时间(ms)
};
extern const Cycle cycle;

struct Hardware {
    struct Odometer {
        const double pos_wheel_radius = 1.375;           // 定位轮半径(inch)
        const double IMU_heading_coef = 3600.0 / 3598.5; // 陀螺仪航向校正系数
        const Vector offset = Vector(-2.6016, -0.1173);
        struct Orthogonal {
            // 定位轮正方向：车沿该方向运动时编码器读数增加
            // 定位轮安装角度：车头沿着x轴正方向摆放时与x轴正方向的夹角，CCW-positive
            // 测量方式：将车沿x轴正方向移动一定距离，记录编码器读数，用反三角函数计算夹角
            const double left_encoder_angle = acos((82.98) / (24 * 5));
            const double right_encoder_angle = -acos((84.55) / (24 * 5));
            const double den = sin(right_encoder_angle - left_encoder_angle);
            // 定位轮所在直线交点（直接定位对象）关于车中心点的偏移，x正方向为车头
        } orthogonal;
        struct Differential {
            const double chassis_width = 12.0; // 底盘两轮间距(inch)
            const double chassis_dist_per_deg =
                24 / 1280.0; // 底盘每转一度行进的距离(inch)
        } differential;
    } odometer;

    struct DistanceThreshold {
        const int exit = 50;     // 球在出口距离阈值
        const int entrance = 30; // 球在入口距离阈值
    } distance_threshold;
};
extern const Hardware hardware;

struct Config {
#ifdef XDRIVE
    struct Reach {
        int max_speed = 100;   // 最大速度(%)
        bool if_hold = true;   // 到达时是否保持制动
        int time_limit = 2000; // ms
        int jump_time = 100;   // ms
        PIDParam PID_trans = PIDParam(0.0, 0.0, 0.0, 10, 20);
        PIDParam PID_rot = PIDParam(0.0, 0.0, 0.0, 10, 20);
        double trans_tol = 2.0;         // inch
        double trans_speed_tol = 0.001; // inch/ms
        double rot_tol = 2.0;           // deg
        double rot_speed_tol = 0.001;   // deg/ms
    } reach;

    struct Trace {
        int max_speed = 100;          // 最大速度(%)
        bool if_hold = true;          // 到达时是否保持制动
        int time_limit = 2000;        // ms
        int jump_time = 100;          // ms
        bool is_terminal = false;     // 是否为终点路径段
        double look_ahead_dist = 4.0; // 默认前瞻距离(inch)
        PIDParam PID_trans = PIDParam(0.0, 0.0, 0.0, 10, 20);
        PIDParam PID_rot = PIDParam(0.0, 0.0, 0.0, 10, 20);
        PIDParam PID_back = PIDParam(0.0, 0.0, 0.0, 10, 20);
        double trans_tol = 2.0;         // inch
        double trans_speed_tol = 0.001; // inch/ms
        double rot_tol = 2.0;           // deg
        double rot_speed_tol = 0.001;   // deg/ms
    } trace;
#endif

#ifdef STRAIGHT
    struct Forward {
        int max_speed = 100;     // 最大速度(%)
        bool if_hold = true;     // 到达时是否保持制动
        int time_limit = 2000;   // ms
        int jump_time = 100;     // ms
        bool is_terminal = true; // 是否视为终点动作
        PIDParam PID_fb = PIDParam(4.3, 0.0, 0.6, 20, 20);
        PIDParam PID_rot = PIDParam(3.0, 0.0, 10.0, 10, 20);
        double fb_tol = 0.5;         // inch
        double fb_speed_tol = 0.003; // inch/ms
        double rot_tol = 1.0;        // deg
        double rot_speed_tol = 0.01; // deg/ms
    } forward;

    struct Turn {
        int max_speed = 100;     // 最大速度(%)
        bool if_hold = true;     // 到达时是否保持制动
        int time_limit = 2000;   // ms
        int jump_time = 100;     // ms
        bool is_terminal = true; // 是否视为终点动作
        PIDParam PID_rot = PIDParam(0.605, 0.003, 4.5, 5, 10);
        PIDParam PID_fb = PIDParam(0.3, 0.0, 0.0, 10, 20);
        double rot_tol = 1.0;         // deg
        double rot_speed_tol = 0.002; // deg/ms
        double adj_tol = 3.0;         // deg
        double adj_speed_tol = 0.01;  // deg/ms
    } turn;

    struct Pose {
        int max_speed = 100;     // 最大速度(%)
        bool if_hold = true;     // 到达时是否保持制动
        int time_limit = 2000;   // ms
        int jump_time = 100;     // ms
        bool is_terminal = true; // 是否视为终点动作
        PIDParam PID_fb_far = PIDParam(3.0, 0.0, 0.0, 5, 10);
        PIDParam PID_rot_far = PIDParam(1.3, 0.0, 10.0, 5, 5);
        PIDParam PID_fb_close_bad = PIDParam(4.0, 0.0, 0.6, 20, 20);
        PIDParam PID_rot_close_bad = PIDParam(2.0, 0.0, 12.0, 10, 20);
        PIDParam PID_fb_close_good = PIDParam(5.0, 0.0, 0.6, 20, 20);
        PIDParam PID_rot_close_good = PIDParam(2.0, 0.0, 10.0, 10, 20);
        double close_dis_th = 7.5;      // inch
        double close_heading_th = 20.0; // deg
        double leading_factor = 0.5;    // carrot point leading factor
        double fb_speed_min = 0.0;      // inch/ms
        double fb_tol = 0.5;            // inch
        double fb_speed_tol = 1.0;      // inch/ms
        double rot_tol = 1.0;           // deg
        double rot_speed_tol = 1.0;     // deg/ms
    } pose;
#endif
};
extern Config config;

#endif // PARAMS_H
