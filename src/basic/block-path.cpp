#include "basic/block-path.h"
#include "basic/driving.h"
#include "basic/robot-config.h"
#include "queue"
#include <iostream>
using namespace vex;
using namespace std;

// STOP / INTAKE / LCG / UCG / LG / EXPANSION
static PathEvent path_event = STOP, last_path_event = STOP;
static float speed_coef = 1.f;
bool is_roller_stuck = false, last_is_roller_stuck = false;
int roller_reverse_duration = 200; // ms
int roller_reverse_speed = 100;    // %
int roller_reverse_cycle = 400;    // ms
bool first_time = true;

timer roller_refresh_timer;
timer roller_stuck_timer;

void spinIntaker(float pct) {
    if (pct) {
        Motors_Intaker.spin(directionType::fwd, 127 * pct, voltageUnits::mV);
    } else {
        Motors_Intaker.stop(coast);
    }
}

void spinShooter(float pct) {
    if (pct)
        Motor_Shooter.spin(directionType::fwd, pct, velocityUnits::pct);
    else if (first_time)
        Motor_Shooter.stop(hold);
    else
        Motor_Shooter.stop(coast);
}

void spinRoller(float pct) {
    if (pct)
        Motor_Roller.spin(directionType::fwd, pct, velocityUnits::pct);
    else if (first_time)
        Motor_Roller.stop(hold);
    else
        Motor_Roller.stop(coast);
}

bool blockAtEntrance() {
    return Distance_Entrance.objectDistance(mm) < hardware.distance_threshold.entrance;
}
bool blockAtExit() {
    return Distance_Exit.objectDistance(mm) < hardware.distance_threshold.exit;
}

bool wrongBlockDetected() {
    if (!blockAtExit()) {
        return false;
    } else if (current_color == RED) {
        return Optical_Exit.color() == vex::blue;
    } else if (current_color == BLUE) {
        return Optical_Exit.color() == vex::red;
    } else {
        return false;
    }
}

void setPathEvent(PathEvent event) { path_event = event; }
void setPathSpeedCoef(float coef) { speed_coef = coef; }

void updateBlockPath() {
    roller_refresh_timer.reset();
    roller_stuck_timer.reset();
    int last_roller_degree = Motor_Roller.position(deg);
    int last_shooter_degree = Motor_Shooter.position(deg);
    bool roller_first_time = true;
    bool roller_stopped = false;
    bool shooter_stopped = false;
    while (true) {
        // first time flags
        if (roller_stuck_timer.time() > 200) {
            roller_first_time = false;
        }
        if (last_path_event != path_event) {
            first_time = false;
        }

        // roller stuck detection
        if (roller_refresh_timer.time() > roller_reverse_cycle) {
            roller_refresh_timer.reset();
            roller_stopped = abs(Motor_Roller.position(deg) - last_roller_degree) < 10;
            shooter_stopped = abs(Motor_Shooter.position(deg) - last_shooter_degree) < 10;
            if (path_event != 0 && (roller_stopped || shooter_stopped)) {
                is_roller_stuck = true;
            } else {
                is_roller_stuck = false;
            }
            last_roller_degree = Motor_Roller.position(deg);
            last_shooter_degree = Motor_Shooter.position(deg);
        }

        // 设置反转计时
        if (!last_is_roller_stuck && is_roller_stuck) {
            roller_stuck_timer.reset();
            cout << "Roller stuck, retrying..." << endl;
        }
        // 设置反转
        bool is_roller_normal = false;
        if (!roller_first_time && roller_stuck_timer.time() < roller_reverse_duration &&
            last_path_event == path_event) {
            if (path_event == 2) {
                spinRoller(roller_reverse_speed);
            } else {
                spinRoller(-roller_reverse_speed);
            }
            is_roller_stuck = false;
        } else {
            is_roller_normal = true;
        }

        // 正常运动模式
        if (is_roller_normal) {
            switch (path_event) {
                case STOP:
                    spinIntaker(0);
                    spinRoller(0);
                    spinShooter(0);
                    break;
                case INTAKE:
                    spinIntaker(100);
                    spinRoller(40);
                    spinShooter(-30);
                    Piston_Shooter.set(true);
                    break;
                case LCG:
                    spinIntaker(-50 * speed_coef);
                    spinRoller(-100 * speed_coef);
                    spinShooter(-20);
                    Piston_Shooter.set(true);
                    break;
                case UCG:
                    spinIntaker(100);
                    spinRoller(80 * speed_coef);
                    spinShooter(-100 * speed_coef);
                    Piston_Shooter.set(false);
                    break;
                case LG:
                    spinIntaker(100);
                    spinRoller(80 * speed_coef);
                    spinShooter(100 * speed_coef);
                    Piston_Shooter.set(true);
                    break;
                case EXPANSION:
                    spinShooter(30);
                    spinRoller(40);
                    spinIntaker(100);
                    Piston_Shooter.set(true);
                    break;
            }
        }

        // 更新状态变量
        last_is_roller_stuck = is_roller_stuck;
        last_path_event = path_event;

        this_thread::sleep_for(cycle.driver);
    }
}
