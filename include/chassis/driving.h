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