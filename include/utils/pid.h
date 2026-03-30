#ifndef PID_H
#define PID_H

#include "utils/geometry.h"
#include "vex.h"
using namespace vex;

/**
 * @brief PID 参数结构体
 * @param kP_ 比例系数
 * @param kI_ 积分系数
 * @param kD_ 微分系数
 * @param I_th_ 启动积分项误差阈值
 * @param I_max_ 积分项饱和值
 * @param sign_flip_clear_ 是否在误差符号翻转时清零积分项
 */
struct PIDParam {
    double kP_, kI_, kD_;
    double I_th_, I_max_;
    bool sign_flip_clear_;
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
    double err_crt_, err_prev_;
    double err_dvt_;
    double err_int_;
    bool first_update_ = true;
    double P_, I_, D_;
    double output_;

  public:
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
    void setITh(double I_th) { I_th_ = I_th; }
    void setIMax(double I_max) { I_max_ = I_max; }
    void setSignFlipClear(bool sign_flip_clear) { sign_flip_clear_ = sign_flip_clear; }

    /**
     * @brief 根据当前误差更新PID控制器变量
     * @param err_crt 当前误差（目标 - 实际）
     */
    void update(double err_crt);
    double getOutput() const { return output_; }
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
    Vector P_, I_, D_;
    Vector output_;

  public:
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
    void setITh(double I_th) { I_th_ = I_th; }
    void setIMax(double I_max) { I_max_ = I_max; }
    void setSignFlipClear(bool sign_flip_clear) { sign_flip_clear_ = sign_flip_clear; }

    /**
     * @brief 根据当前向量误差计算输出向量
     * @param err_crt 目标-当前的误差向量
     * @return 校正向量
     */
    void update(const Vector& err_crt);
    const Vector& getOutput() const { return output_; }
    /**
     * @brief 重置内部状态
     */
    void reset();
};

#endif // PID_H