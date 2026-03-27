#include "debug.h"
#include "auto-functions/motion.h"
#include "autonomous.h"
#include "basic/params.h"
#include "basic/robot-config.h"
#include "ui/base.h"
#include "basic/positioning.h"
#include "vex.h"
using namespace vex;

/*********************** 调试键位表 ***********************/
// RE-INIT & CALIBRATION
#define BTN_REINIT (Controller.ButtonUp.pressing())
// PID_DEBUG
#define BTN_DEBUG_FWD (Controller.ButtonX.pressing())
#define BTN_DEBUG_BWD (Controller.ButtonB.pressing())
#define BTN_DEBUG_ROTR (Controller.ButtonA.pressing())
#define BTN_DEBUG_ROTL (Controller.ButtonY.pressing())
#define BTN_DEBUG_POSEL (Controller.ButtonL1.pressing())
#define BTN_DEBUG_POSER (Controller.ButtonR1.pressing())
#define BTN_DEBUG_CONTINUOUS_POSE (Controller.ButtonDown.pressing())
// ROUTE_DEBUG
#define BTN_DEBUG_ROUTE (Controller.ButtonDown.pressing())
/*********************************************************/

void debugControl() {
    while (true) {
        if (current_mode != COMPETITION) {
            if (BTN_REINIT) {
                botInit();
            }
        }
        if (current_mode == PID_DEBUG) {
            chassisDebug();
        } else if (current_mode == ROUTE_DEBUG) {
            if (BTN_DEBUG_ROUTE) {
                autonomous();
            }
        }
        this_thread::sleep_for(cycle.driver);
    }
}

void chassisDebug() {
#ifdef XDRIVE
    if (BTN_DEBUG_FWD) {
        Point cur_pos = myPosition.getCenterPos();
        double cur_heading = myPosition.getHeading();
        reach(cur_pos + Vector(0, 24), cur_heading,
              Config{.reach = Config::Reach{.time_limit = 2000,
                                            .PID_trans = config.reach.PID_trans,
                                            .PID_rot = config.reach.PID_rot}});
    }
    if (BTN_DEBUG_BWD) {
        Point cur_pos = myPosition.getCenterPos();
        double cur_heading = myPosition.getHeading();
        reach(cur_pos + Vector(0, -24), cur_heading,
              Config{.reach = Config::Reach{.time_limit = 2000,
                                            .PID_trans = config.reach.PID_trans,
                                            .PID_rot = config.reach.PID_rot}});
    }
    if (BTN_DEBUG_ROTL) {
        Point cur_pos = myPosition.getCenterPos();
        double cur_heading = myPosition.getHeading();
        reach(cur_pos, cur_heading + 90,
              Config{.reach = Config::Reach{.time_limit = 2000,
                                            .PID_trans = config.reach.PID_trans,
                                            .PID_rot = config.reach.PID_rot}});
    }
    if (BTN_DEBUG_ROTR) {
        Point cur_pos = myPosition.getCenterPos();
        double cur_heading = myPosition.getHeading();
        reach(cur_pos, cur_heading - 90,
              Config{.reach = Config::Reach{.time_limit = 2000,
                                            .PID_trans = config.reach.PID_trans,
                                            .PID_rot = config.reach.PID_rot}});
    }
#endif
#ifdef STRAIGHT
    if (BTN_DEBUG_FWD) {
        setParamProfile(PROFILE_FWD);
        myDrive.setImportantFactor(ROT);
        pidForward(40, 90,
                   Config{.forward = Config::Forward{.time_limit = 2000,
                                                     .PID_fb = config.forward.PID_fb,
                                                     .PID_rot = config.forward.PID_rot}});
        myDrive.setImportantFactor(NEITHER);
        myDrive.setBrakeType(coast);
    }
    if (BTN_DEBUG_BWD) {
        setParamProfile(PROFILE_FWD);
        myDrive.setImportantFactor(ROT);
        pidForward(-40, 90,
                   Config{.forward = Config::Forward{.time_limit = 2000,
                                                     .PID_fb = config.forward.PID_fb,
                                                     .PID_rot = config.forward.PID_rot}});
        myDrive.setImportantFactor(NEITHER);
        myDrive.setBrakeType(coast);
    }
    if (BTN_DEBUG_ROTL) {
        setParamProfile(PROFILE_TURN);
        myDrive.setImportantFactor(ROT);
        turnTo(myPosition.getHeading() + 90,
               Config{.turn = Config::Turn{.time_limit = 2000,
                                           .PID_rot = config.turn.PID_rot,
                                           .PID_fb = config.turn.PID_fb}});
        myDrive.setImportantFactor(NEITHER);
        myDrive.setBrakeType(coast);
    }
    if (BTN_DEBUG_ROTR) {
        setParamProfile(PROFILE_TURN);
        myDrive.setImportantFactor(ROT);
        turnTo(myPosition.getHeading() - 180,
               Config{.turn = Config::Turn{.time_limit = 2000,
                                           .PID_rot = config.turn.PID_rot,
                                           .PID_fb = config.turn.PID_fb}});
        myDrive.setImportantFactor(NEITHER);
        myDrive.setBrakeType(coast);
    }
    if (BTN_DEBUG_POSEL) {
        setParamProfile(PROFILE_POSE_FAR);
        myDrive.setImportantFactor(ROT);
        moveToPose(Point(-66.5, 38), -90, true,
                   Config{.pose = Config::Pose{
                              .time_limit = 5000,
                              .PID_fb_far = config.pose.PID_fb_far,
                              .PID_rot_far = config.pose.PID_rot_far,
                              .PID_fb_close_bad = config.pose.PID_fb_close_bad,
                              .PID_rot_close_bad = config.pose.PID_rot_close_bad,
                              .PID_fb_close_good = config.pose.PID_fb_close_good,
                              .PID_rot_close_good = config.pose.PID_rot_close_good}});
        myDrive.setImportantFactor(NEITHER);
        myDrive.setBrakeType(coast);
    }
    if (BTN_DEBUG_POSER) {
        setParamProfile(PROFILE_POSE_FAR);
        myDrive.setImportantFactor(ROT);
        moveToPose(Point(30.0, 38), -90, true,
                   Config{.pose = Config::Pose{
                              .time_limit = 5000,
                              .PID_fb_far = config.pose.PID_fb_far,
                              .PID_rot_far = config.pose.PID_rot_far,
                              .PID_fb_close_bad = config.pose.PID_fb_close_bad,
                              .PID_rot_close_bad = config.pose.PID_rot_close_bad,
                              .PID_fb_close_good = config.pose.PID_fb_close_good,
                              .PID_rot_close_good = config.pose.PID_rot_close_good}});
        myDrive.setImportantFactor(NEITHER);
        myDrive.setBrakeType(coast);
    }
    if (BTN_DEBUG_CONTINUOUS_POSE) {
        setParamProfile(PROFILE_POSE_FAR);
        myDrive.setImportantFactor(ROT);
        moveToPose(Point(8.0, 47), 90, false,
                   Config{.pose = Config::Pose{
                              .time_limit = 3000,
                              .jump_time = 0,
                              .is_terminal = false,
                              .PID_fb_far = config.pose.PID_fb_far,
                              .PID_rot_far = config.pose.PID_rot_far,
                              .PID_fb_close_bad = config.pose.PID_fb_close_bad,
                              .PID_rot_close_bad = config.pose.PID_rot_close_bad,
                              .PID_fb_close_good = config.pose.PID_fb_close_good,
                              .PID_rot_close_good = config.pose.PID_rot_close_good,
                              .fb_speed_min = 30}});
        moveToPose(Point(-14, 95), 180, false,
                   Config{.pose = Config::Pose{
                              .time_limit = 5000,
                              .PID_fb_far = config.pose.PID_fb_far,
                              .PID_rot_far = config.pose.PID_rot_far,
                              .PID_fb_close_bad = config.pose.PID_fb_close_bad,
                              .PID_rot_close_bad = config.pose.PID_rot_close_bad,
                              .PID_fb_close_good = config.pose.PID_fb_close_good,
                              .PID_rot_close_good = config.pose.PID_rot_close_good}});
        myDrive.setImportantFactor(NEITHER);
        myDrive.setBrakeType(coast);
    }
#endif
}
