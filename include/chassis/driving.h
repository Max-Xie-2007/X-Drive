#ifndef DRIVING_H
#define DRIVING_H

#include "utils/geometry.h"
#include "utils/params.h"
#include "vex.h"

using namespace vex;

enum DriveMethod { VOLT, PCT };
enum DriveFactor { TRANS, ANGULAR, NEITHER };

/**
 * @brief 底盘控制线程函数
 */
void updateChassis();

class XDrive { // 菱底
  private:
    motor* motors_lf_;
    motor* motors_lb_;
    motor* motors_rf_;
    motor* motors_rb_;
    int motor_count_;
    // Absolute speeds
    Vector abs_auton_trans_;
    Vector abs_driver_trans_;
    // Relative speeds
    Vector rel_auton_trans_;
    Vector rel_driver_trans_;

    double auton_rot_speed_;
    double driver_rot_speed_;

    DriveMethod current_drive_method_;
    brakeType current_brake_type_;
    DriveFactor important_factor_ = NEITHER;

  public:
    /**
     * @brief 构造 XDrive
     * @param LF 左前电机组
     * @param LB 左后电机组
     * @param RF 右前电机组
     * @param RB 右后电机组
     */
    XDrive(motor* motors_lf, motor* motors_lb, motor* motors_rf, motor* motors_rb,
           int motor_count)
        : motors_lf_(motors_lf), motors_lb_(motors_lb), motors_rf_(motors_rf),
          motors_rb_(motors_rb), motor_count_(motor_count), abs_auton_trans_(0),
          abs_driver_trans_(0), rel_auton_trans_(0), rel_driver_trans_(0),
          auton_rot_speed_(0), driver_rot_speed_(0), current_drive_method_(VOLT),
          current_brake_type_(coast) {}

    /**
     * @brief 设置自动阶段的全局平移速度与旋转速度
     * @param auton_trans_speed 全局平移速度向量（%）
     * @param auton_rot_speed 旋转速度（%）
     */
    void setAbsAuton(Vector auton_trans_speed, double auton_rot_speed);
    /**
     * @brief 设置遥控阶段的全局平移速度与旋转速度
     * @param auton_trans_speed 全局平移速度向量（%）
     * @param driver_rot_speed 旋转速度（%）
     */
    void setAbsDriver(Vector auton_trans_speed, double driver_rot_speed);
    /**
     * @brief 设置自动阶段的机体坐标平移速度与旋转速度
     * @param auton_trans_speed 机体坐标平移速度向量（%）
     * @param auton_rot_speed 旋转速度（%）
     */
    void setRelAuton(Vector auton_trans_speed, double auton_rot_speed);
    /**
     * @brief 设置遥控阶段的机体坐标平移速度与旋转速度
     * @param auton_trans_speed 机体坐标平移速度向量（%）
     * @param driver_rot_speed 旋转速度（%）
     */
    void setRelDriver(Vector auton_trans_speed, double driver_rot_speed);
    /**
     * @brief 设置驱动方式（电压或百分比）
     * @param method 驱动方式，VOLT 或 PCT
     */
    void setDriveMethod(DriveMethod method = VOLT);
    /**
     * @brief 设置制动类型
     * @param type 制动类型，coast / hold / brake
     */
    void setBrakeType(brakeType type = coast);
    /**
     * @brief 设置驱动优先级（用于功率和超过 100% 的情况）
     * @param factor 优先级因子（TRANS / ANGULAR / NEITHER）
     */
    void setPriorFactor(DriveFactor factor);

    /**
     * @brief 设置机器人的驱动
     * @param important_factor_ 决定100饱和值中哪个部分占满原有数值
     */
    void drive();
    /**
     * @brief 停止机器人运动
     * @param type 制动类型
     */
    void stop(brakeType type = brakeType::hold);
};

#endif // DRIVING_H