#include "utils/params.h"

const Cycle cycle = {.auton = 5,
                     .position = 5,
                     .driver_control = 20,
                     .brain_ui_update = 100,
                     .controller_ui_update = 100};
const Hardware hardware = {.odometer = {.rotation_dist_per_deg = 4 * 24 / 7348.97,
                                        .IMU_heading_coef = 3600.0 / (9 * 360 + 344.42),
                                        .IMU_damping_th = 0.01,
                                        .left_encoder_angle = -acos(67.9 / (24 * 4)),
                                        .right_encoder_angle = acos(65.7 / (24 * 4)),
                                        .offset = Vector(6.15, 0)}};

ReachCfg default_reach_cfg;
TraceCfg default_trace_cfg;

PIDParam default_reach_translational = {.kP_ = 10.0,
                                        .kI_ = 0.0,
                                        .kD_ = 0.0,
                                        .I_th_ = 0.0,
                                        .I_max_ = 0.0,
                                        .sign_flip_clear_ = true};
PIDParam default_reach_angular = {.kP_ = 2.0,
                                  .kI_ = 0.0,
                                  .kD_ = 0.0,
                                  .I_th_ = 0.0,
                                  .I_max_ = 0.0,
                                  .sign_flip_clear_ = true};
PIDParam default_trace_translational = {.kP_ = 0.8,
                                        .kI_ = 0.0,
                                        .kD_ = 0.0,
                                        .I_th_ = 0.0,
                                        .I_max_ = 0.0,
                                        .sign_flip_clear_ = true};
PIDParam default_trace_angular = {.kP_ = 0.8,
                                  .kI_ = 0.0,
                                  .kD_ = 0.0,
                                  .I_th_ = 0.0,
                                  .I_max_ = 0.0,
                                  .sign_flip_clear_ = true};
PIDParam default_trace_deviational = {.kP_ = 0.8,
                                      .kI_ = 0.0,
                                      .kD_ = 0.0,
                                      .I_th_ = 0.0,
                                      .I_max_ = 0.0,
                                      .sign_flip_clear_ = true};

ExitConditionOptions default_translational_ec_opt = {
    .err_tol_ = 0.5, .dvt_tol_ = 0.003, .jump_time_ = 50, .exit_on_overshoot_ = false};
ExitConditionOptions default_angular_ec_opt = {
    .err_tol_ = 1.0, .dvt_tol_ = 0.003, .jump_time_ = 50, .exit_on_overshoot_ = false};