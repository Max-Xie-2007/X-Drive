#ifndef EXIT_CONDITION_H
#define EXIT_CONDITION_H

#include "utils/calc.h"
#include "vex.h"
#include <cmath>

/**
 * @brief 退出条件类，用于判断运动是否达到预期目标以停止运动
 * @param err_tol_ 误差阈值
 * @param dvt_tol_ 误差变化率阈值（/ms）
 * @param jump_time_ 持续满足条件的时间（ms）
 * @param exit_on_overshoot_ 是否在产生过冲时立即退出
 */
struct ExitConditionOptions {
    double err_tol_ = 0.5;
    double dvt_tol_ = 0.003;
    int jump_time_ = 50;
    bool exit_on_overshoot_ = false;
};

class ExitCondition1D : public ExitConditionOptions {
  private:
    int init_err_sign_;
    vex::timer jump_timer_;
    bool reached_ = false;

  public:
    explicit ExitCondition1D(const ExitConditionOptions& opt);

    void setExitOnOvershoot(bool enabled) { exit_on_overshoot_ = enabled; }
    /**
     * @brief 重置退出条件状态，通常在开始一个新的运动时调用
     * @param init_err 初始误差（目标 - 实际），用于确定误差符号以判断过冲
     */
    void reset(double init_err);
    /**
     * @brief 判断是否满足退出条件
     * @param err 当前误差（目标 - 实际）
     * @param dvt 当前误差变化率（/ms）
     * @return 满足退出条件返回 true，否则返回 false
     * @note 若 exit_on_overshoot_ 启用且产生过冲则立即返回 true
     */
    void update(double err = 0, double dvt = 0);
    bool reached() const { return reached_; }
};

class ExitCondition2D : public ExitConditionOptions {
  private:
    Vector init_err_vec_;
    vex::timer jump_timer_;
    bool reached_ = false;

  public:
    explicit ExitCondition2D(const ExitConditionOptions& opt);

    void setErrorTol(double err_tol) { err_tol_ = err_tol; }
    void setExitOnOvershoot(bool enabled) { exit_on_overshoot_ = enabled; }
    /**
     * @brief 重置退出条件状态，通常在开始一个新的运动时调用
     * @param init_err_vec 初始误差向量（目标 - 实际），用于判断过冲
     */
    void reset(Vector init_err_vec = Vector());
     /**
     * @brief 判断是否满足退出条件
     * @param err_vec 当前误差向量（目标 - 实际）
     * @param dvt_vec 当前误差变化率向量（/ms）
     * @return 满足退出条件返回 true，否则返回 false
     * @note 若 exit_on_overshoot_ 启用且产生过冲则立即返回 true
     */
    void update(Vector err_vec = Vector(), Vector dvt_vec = Vector());
    bool reached() const { return reached_; }
};


#endif // EXIT_CONDITION_H