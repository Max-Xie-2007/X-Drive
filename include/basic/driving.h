#ifndef DRIVING_H
#define DRIVING_H
#include "basic/params.h"
#include "utils/geometry.h"
#include "vex.h"

using namespace vex;

enum DriveMethod { VOLT, PCT };
enum DriveFactor { TRANS, ROT, NEITHER };

void updateChassis();

#ifdef XDRIVE

class XDrive { // 菱底
  private:
    motor_group motors_lf_, motors_lb_, motors_rf_, motors_rb_;
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
    XDrive(motor_group& motors_lf, motor_group& motors_lb, motor_group& motors_rf,
           motor_group& motors_rb);

    void setAbsAuton(Vector auton_trans_speed, double auton_rot_speed);
    void setAbsDriver(Vector auton_trans_speed, double driver_rot_speed);
    void setRelAuton(Vector auton_trans_speed, double auton_rot_speed);
    void setRelDriver(Vector auton_trans_speed, double driver_rot_speed);
    void setDriveMethod(DriveMethod method = VOLT);
    void setBrakeType(brakeType type = coast);
    void setImportantFactor(DriveFactor factor);

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

#endif

#ifdef STRAIGHT

class Straight { // 直底
  private:
    motor_group motors_left_, motors_right_;
    double auton_fb_speed_;
    double auton_rot_speed_;
    double driver_fb_speed_;
    double driver_rot_speed_;

    DriveMethod current_drive_method_;
    brakeType current_brake_type_;
    DriveFactor important_factor_ = NEITHER;

  public:
    Straight(motor_group& motors_left, motor_group& motors_right);
    /**
     * @brief 设置机器人的自动驾驶速度
     * @param fb 前进后退速度（百分比，正前进负后退）
     * @param rot 旋转速度（百分比，正逆时针负顺时针）
     */
    void setAuton(double fb, double rot);
    /**
     * @brief 设置机器人的手动驾驶速度
     * @param fb 前进后退速度（百分比，正前进负后退）
     * @param rot 旋转速度（百分比，正逆时针负顺时针）
     */
    void setDriver(double fb, double rot);
    void setDriveMethod(DriveMethod method = VOLT);
    void setBrakeType(brakeType type = coast);
    void setImportantFactor(DriveFactor factor);

    void resetPosL();
    void resetPosR();
    void resetPos();
    double getPosL() { return motors_left_.position(rotationUnits::deg); }
    double getPosR() { return motors_right_.position(rotationUnits::deg); }
    double getPosAvg() { return (getPosL() + getPosR()) / 2; }
    double getDisL() { return getPosL() * hardware.odometer.differential.chassis_dist_per_deg; }
    double getDisR() { return getPosR() * hardware.odometer.differential.chassis_dist_per_deg; }
    double getDisAvg() { return (getDisL() + getDisR()) / 2; }

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

#endif

#endif // DRIVING_H