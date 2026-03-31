#include "utils/params.h"

const Cycle cycle;
const Hardware hardware;

ReachCfg default_reach_cfg;
TraceCfg default_trace_cfg;

PIDParam default_reach_translational = {.kP_ = 0.8,
                                        .kI_ = 0.0,
                                        .kD_ = 0.0,
                                        .I_th_ = 0.0,
                                        .I_max_ = 0.0,
                                        .sign_flip_clear_ = true};
PIDParam default_reach_angular = {.kP_ = 0.8,
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
PIDParam default_trace_return = {.kP_ = 0.8,
                                 .kI_ = 0.0,
                                 .kD_ = 0.0,
                                 .I_th_ = 0.0,
                                 .I_max_ = 0.0,
                                 .sign_flip_clear_ = true};

ExitConditionOptions default_translational_ec_opt = {
    .err_tol = 0.5, .dvt_tol = 0.003, .jump_time = 50, .exit_on_overshoot = false};
ExitConditionOptions default_angular_ec_opt = {
    .err_tol = 1.0, .dvt_tol = 0.003, .jump_time = 50, .exit_on_overshoot = false};