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
PIDParam default_trace_back = {.kP_ = 0.8,
                               .kI_ = 0.0,
                               .kD_ = 0.0,
                               .I_th_ = 0.0,
                               .I_max_ = 0.0,
                               .sign_flip_clear_ = true};
