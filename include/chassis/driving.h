#ifndef DRIVING_H
#define DRIVING_H
#include "utils/geometry.h"
#include "utils/params.h"
#include "vex.h"

using namespace vex;

enum DriveMethod { VOLT, PCT };
enum DriveFactor { TRANS, ANGULAR, NEITHER };

void updateChassis();

class XDrive { // 菱底
  private:
    motor motor_lf_, motor_lb_, motor_rf_, motor_rb_;
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
    XDrive(motor& motor_lf, motor& motor_lb, motor& motor_rf, motor& motor_rb);

    void setAbsAuton(Vector auton_trans_speed, double auton_rot_speed);
    void setAbsDriver(Vector auton_trans_speed, double driver_rot_speed);
    void setRelAuton(Vector auton_trans_speed, double auton_rot_speed);
    void setRelDriver(Vector auton_trans_speed, double driver_rot_speed);
    void setDriveMethod(DriveMethod method = VOLT);
    void setBrakeType(brakeType type = coast);
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