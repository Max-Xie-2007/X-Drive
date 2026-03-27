#ifndef PID_H
#define PID_H

#include "utils/geometry.h"
#include "vex.h"
using namespace vex;

class PIDParam {
  public:
    double kP_, kI_, kD_;
    double I_th_, I_max_;

    PIDParam(const double& kP = 0, const double& kI = 0, const double& kD = 0,
             const double& I_th = 0, const double& I_max = 0)
        : kP_(kP), kI_(kI), kD_(kD), I_th_(I_th), I_max_(I_max) {}
};
/**
 * @brief 单变量 PID 控制器
 *
 * 用于控制单一标量量（例如朝向误差）。
 */
class PID1D : public PIDParam {
  private:
    timer track_timer_;
    int last_time_;
    int dt_ = 0;
    timer jump_timer_;
    int jump_time_ = 100;
    double err_crt_, err_prev_;
    double err_dvt_;
    double err_int_;
    bool first_update_ = true;
    bool clear_I_ = false;
    double err_tol_, dvt_tol_;
    double P_, I_, D_;

  public:
    bool reached_;
    double output_;
    /**
     * @brief 构造一个 1D PID 控制器
     * @param kP 比例系数
     * @param kI 积分系数
     * @param kD 微分系数
     * @param I_th_ 启动积分项误差阈值
     * @param I_max_ 积分项饱和值
     */
    explicit PID1D(const PIDParam& param);
    void setParam(const PIDParam& param);
    void setCoeff(double kP, double kI, double kD);
    void setJumpTime(int jump_time) { jump_time_ = jump_time; }
    void setITh(double I_th) { I_th_ = I_th; }
    void setIMax(double I_max) { I_max_ = I_max; }
    void setErrTol(double err_tol) { err_tol_ = err_tol; }
    void setDvtTol(double dvt_tol) { dvt_tol_ = dvt_tol; }
    void setClearI(bool clear_I) { clear_I_ = clear_I; }

    /**
     * @brief 根据当前误差更新PID控制器变量
     * @param err_crt 当前误差（目标 - 实际）
     */
    void update(double err_crt);
    /**
     * @brief 重置内部积分/历史状态
     */
    void reset();
};

/**
 * @brief 二维 PID 控制器
 *
 * 对向量误差进行处理，输出为二维校正向量。
 */
class PID2D : public PIDParam {
  private:
    timer track_timer_;
    int last_time_;
    int dt_ = 0;
    Vector err_crt_, err_prev_;
    Vector err_dvt_;
    Vector err_int_;
    bool first_update_ = true;
    timer jump_timer_;
    int jump_time_ = 100;
    bool clear_I_ = false;
    double err_tol_, dvt_tol_;
    Vector P_, I_, D_;

  public:
    Vector output_;
    bool reached_;
    /**
     * @brief 构造一个 2D PID 控制器
     * @param kP 比例系数
     * @param kI 积分系数
     * @param kD 微分系数
     * @param I_th_ 启动积分项误差阈值
     * @param I_max_ 积分项饱和值
     */
    explicit PID2D(const PIDParam& param);
    void setParam(const PIDParam& param);
    void setCoeff(double kP, double kI, double kD);
    void setJumpTime(int jump_time) { jump_time_ = jump_time; }
    void setITh(double I_th) { I_th_ = I_th; }
    void setIMax(double I_max) { I_max_ = I_max; }
    void setErrTol(double err_tol) { err_tol_ = err_tol; }
    void setDvtTol(double dvt_tol) { dvt_tol_ = dvt_tol; }
    void setClearI(bool clear_I) { clear_I_ = clear_I; }

    /**
     * @brief 根据当前向量误差计算输出向量
     * @param err_crt 目标-当前的误差向量
     * @return 校正向量
     */
    void update(const Vector& err_crt);
    /**
     * @brief 重置内部状态
     */
    void reset();
};

#endif // PID_H