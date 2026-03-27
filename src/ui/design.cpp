#include "ui/design.h"
#include "utils/pid.h"
#include <algorithm>
#include <cmath>

#ifdef STRAIGHT
static PIDParam* selectedFbPid() {
    switch (current_param_profile) {
        case PROFILE_FWD:
            return &config.forward.PID_fb;
        case PROFILE_TURN:
            return &config.turn.PID_fb;
        case PROFILE_POSE_FAR:
            return &config.pose.PID_fb_far;
        case PROFILE_POSE_CLOSE_BAD:
            return &config.pose.PID_fb_close_bad;
        case PROFILE_POSE_CLOSE_GOOD:
            return &config.pose.PID_fb_close_good;
    }
    return &config.forward.PID_fb;
}

static PIDParam* selectedRotPid() {
    switch (current_param_profile) {
        case PROFILE_FWD:
            return &config.forward.PID_rot;
        case PROFILE_TURN:
            return &config.turn.PID_rot;
        case PROFILE_POSE_FAR:
            return &config.pose.PID_rot_far;
        case PROFILE_POSE_CLOSE_BAD:
            return &config.pose.PID_rot_close_bad;
        case PROFILE_POSE_CLOSE_GOOD:
            return &config.pose.PID_rot_close_good;
    }
    return &config.forward.PID_rot;
}
#endif

// Global labels and buttons
Button btn_options(0, 0, 96, 40, "Options", [] { setInterface(OPTIONS); });
Button btn_motors(96, 0, 96, 40, "Motors", [] { setInterface(MOTORS); });
Button btn_devices(2 * 96, 0, 96, 40, "Devices", [] { setInterface(DEVICES); });
Button btn_info(3 * 96, 0, 96, 40, "Info", [] { setInterface(INFO); });
Button btn_params(4 * 96, 0, 96, 40, "Params", [] { setInterface(PARAMS); });

Label lbl_routes(0, 50, 240, 30, "Routes");
Label lbl_color(240, 50, 100, 30, "Color");
Label lbl_mode(340, 50, 140, 30, "Mode");

#ifdef LEFT
Button btn_l1(20, 90, 60, 60, "L1", [] { setRoute(L1); });
Button btn_l2(90, 90, 60, 60, "L2", [] { setRoute(L2); });
Button btn_l3(160, 90, 60, 60, "L3", [] { setRoute(L3); });
#endif
#ifdef RIGHT
Button btn_r1(20, 100, 60, 60, "R1", [] { setRoute(R1); });
Button btn_r2(90, 100, 60, 60, "R2", [] { setRoute(R2); });
Button btn_r3(160, 100, 60, 60, "R3", [] { setRoute(R3); });
#endif
Button btn_skills(20, 160, 200, 60, "SKILLS", [] { setRoute(SKILLS); });

Button btn_red(260, 90, 60, 60, "RED", [] { setColor(RED); });
Button btn_blue(260, 160, 60, 60, "BLUE", [] { setColor(BLUE); });

Button btn_competition(360, 85, 100, 40, "COMP", [] { setMode(COMPETITION); });
Button btn_pid_debug(360, 135, 100, 40, "PID", [] { setMode(PID_DEBUG); });
Button btn_route_debug(360, 185, 100, 40, "ROUTE", [] { setMode(ROUTE_DEBUG); });

#ifdef XDRIVE
Label lbl_trans(0, 75, 30, 60, "TRN");
Label lbl_rot(0, 130, 30, 60, "ROT");
Label lbl_back(0, 185, 30, 60, "BCK");
Label lbl_kp(30, 95, 150, 35, "kP");
Label lbl_ki(180, 95, 150, 35, "kI");
Label lbl_kd(330, 95, 150, 35, "kD");

Label lbl_trans_kp(80, 75, 50, 55, config.trace.PID_trans.kP_);
Label lbl_trans_ki(230, 75, 50, 55, config.trace.PID_trans.kI_);
Label lbl_trans_kd(380, 75, 50, 55, config.trace.PID_trans.kD_);
Label lbl_rot_kp(80, 130, 50, 55, config.trace.PID_rot.kP_);
Label lbl_rot_ki(230, 130, 50, 55, config.trace.PID_rot.kI_);
Label lbl_rot_kd(380, 130, 50, 55, config.trace.PID_rot.kD_);
Label lbl_back_kp(80, 185, 50, 55, config.trace.PID_back.kP_);
Label lbl_back_ki(230, 185, 50, 55, config.trace.PID_back.kI_);
Label lbl_back_kd(380, 185, 50, 55, config.trace.PID_back.kD_);

Button btn_trans_kp_add(130, 75, 50, 55, "+", [] {
    config.trace.PID_trans.kP_ += 0.1;
    refresh();
});
Button btn_trans_ki_add(280, 75, 50, 55, "+", [] {
    config.trace.PID_trans.kI_ += 0.1;
    refresh();
});
Button btn_trans_kd_add(430, 75, 50, 55, "+", [] {
    config.trace.PID_trans.kD_ += 0.1;
    refresh();
});
Button btn_trans_kp_minus(30, 75, 50, 55, "-", [] {
    config.trace.PID_trans.kP_ -= 0.1;
    refresh();
});
Button btn_trans_ki_minus(180, 75, 50, 55, "-", [] {
    config.trace.PID_trans.kI_ -= 0.1;
    refresh();
});
Button btn_trans_kd_minus(330, 75, 50, 55, "-", [] {
    config.trace.PID_trans.kD_ -= 0.1;
    refresh();
});
Button btn_rot_kp_add(130, 130, 50, 55, "+", [] {
    config.trace.PID_rot.kP_ += 0.1;
    refresh();
});
Button btn_rot_ki_add(280, 130, 50, 55, "+", [] {
    config.trace.PID_rot.kI_ += 0.1;
    refresh();
});
Button btn_rot_kd_add(430, 130, 50, 55, "+", [] {
    config.trace.PID_rot.kD_ += 0.1;
    refresh();
});
Button btn_rot_kp_minus(30, 130, 50, 55, "-", [] {
    config.trace.PID_rot.kP_ -= 0.1;
    refresh();
});
Button btn_rot_ki_minus(180, 130, 50, 55, "-", [] {
    config.trace.PID_rot.kI_ -= 0.1;
    refresh();
});
Button btn_rot_kd_minus(330, 130, 50, 55, "-", [] {
    config.trace.PID_rot.kD_ -= 0.1;
    refresh();
});
Button btn_back_kp_add(130, 185, 50, 55, "+", [] {
    config.trace.PID_back.kP_ += 0.1;
    refresh();
});
Button btn_back_ki_add(280, 185, 50, 55, "+", [] {
    config.trace.PID_back.kI_ += 0.1;
    refresh();
});
Button btn_back_kd_add(430, 185, 50, 55, "+", [] {
    config.trace.PID_back.kD_ += 0.1;
    refresh();
});
Button btn_back_kp_minus(30, 185, 50, 55, "-", [] {
    config.trace.PID_back.kP_ -= 0.1;
    refresh();
});
Button btn_back_ki_minus(180, 185, 50, 55, "-", [] {
    config.trace.PID_back.kI_ -= 0.1;
    refresh();
});
Button btn_back_kd_minus(330, 185, 50, 55, "-", [] {
    config.trace.PID_back.kD_ -= 0.1;
    refresh();
});
#endif
#ifdef STRAIGHT
Button btn_profile_fwd(0, 45, 96, 35, "FWD", [] { setParamProfile(PROFILE_FWD); });
Button btn_profile_turn(96, 45, 96, 35, "TURN", [] { setParamProfile(PROFILE_TURN); });
Button btn_profile_pose_far(192, 45, 96, 35, "PFAR",
                            [] { setParamProfile(PROFILE_POSE_FAR); });
Button btn_profile_pose_bad(288, 45, 96, 35, "PCBAD",
                            [] { setParamProfile(PROFILE_POSE_CLOSE_BAD); });
Button btn_profile_pose_good(384, 45, 96, 35, "PCGOOD",
                             [] { setParamProfile(PROFILE_POSE_CLOSE_GOOD); });

Label lbl_fb(0, 130, 30, 60, "FB");
Label lbl_rot(0, 185, 30, 60, "ROT");
Label lbl_kp(30, 95, 150, 35, "kP");
Label lbl_ki(180, 95, 150, 35, "kI");
Label lbl_kd(330, 95, 150, 35, "kD");

Label lbl_fb_kp(80, 130, 50, 55, config.forward.PID_fb.kP_);
Label lbl_fb_ki(230, 130, 50, 55, config.forward.PID_fb.kI_);
Label lbl_fb_kd(380, 130, 50, 55, config.forward.PID_fb.kD_);
Label lbl_rot_kp(80, 185, 50, 55, config.forward.PID_rot.kP_);
Label lbl_rot_ki(230, 185, 50, 55, config.forward.PID_rot.kI_);
Label lbl_rot_kd(380, 185, 50, 55, config.forward.PID_rot.kD_);

const double fb_kp_step = 0.1;
const double fb_ki_step = 0.001;
const double fb_kd_step = 1.0;
const double rot_kp_step = 0.1;
const double rot_ki_step = 0.001;
const double rot_kd_step = 1.0;

Button btn_fb_kp_add(130, 130, 50, 55, "+", [] {
    selectedFbPid()->kP_ += fb_kp_step;
    refresh();
});
Button btn_fb_ki_add(280, 130, 50, 55, "+", [] {
    selectedFbPid()->kI_ += fb_ki_step;
    refresh();
});
Button btn_fb_kd_add(430, 130, 50, 55, "+", [] {
    selectedFbPid()->kD_ += fb_kd_step;
    refresh();
});
Button btn_fb_kp_minus(30, 130, 50, 55, "-", [] {
    selectedFbPid()->kP_ -= fb_kp_step;
    refresh();
});
Button btn_fb_ki_minus(180, 130, 50, 55, "-", [] {
    selectedFbPid()->kI_ -= fb_ki_step;
    refresh();
});
Button btn_fb_kd_minus(330, 130, 50, 55, "-", [] {
    selectedFbPid()->kD_ -= fb_kd_step;
    refresh();
});
Button btn_rot_kp_add(130, 185, 50, 55, "+", [] {
    selectedRotPid()->kP_ += rot_kp_step;
    refresh();
});
Button btn_rot_ki_add(280, 185, 50, 55, "+", [] {
    selectedRotPid()->kI_ += rot_ki_step;
    refresh();
});
Button btn_rot_kd_add(430, 185, 50, 55, "+", [] {
    selectedRotPid()->kD_ += rot_kd_step;
    refresh();
});
Button btn_rot_kp_minus(30, 185, 50, 55, "-", [] {
    selectedRotPid()->kP_ -= rot_kp_step;
    refresh();
});
Button btn_rot_ki_minus(180, 185, 50, 55, "-", [] {
    selectedRotPid()->kI_ -= rot_ki_step;
    refresh();
});
Button btn_rot_kd_minus(330, 185, 50, 55, "-", [] {
    selectedRotPid()->kD_ -= rot_kd_step;
    refresh();
});
#endif

void clearBrain() { Brain.Screen.clearScreen(white); }
void clearController() { Controller.Screen.clearScreen(); }

void displayTopBar() {
    Brain.Screen.setPenWidth(1);
    btn_options.render(black, black, current_interface == OPTIONS ? green : white);
    btn_motors.render(black, black, current_interface == MOTORS ? green : white);
    btn_devices.render(black, black, current_interface == DEVICES ? green : white);
    btn_info.render(black, black, current_interface == INFO ? green : white);
    btn_params.render(current_mode == PID_DEBUG ? black : color(128, 128, 128), black,
                      current_interface == PARAMS
                          ? green
                          : (current_mode == PID_DEBUG ? white : color(192, 192, 192)));
}

void displayOptions() {
    Brain.Screen.setPenColor(black);
    Brain.Screen.setPenWidth(1);
    lbl_routes.render();
#ifdef LEFT
    btn_l1.render(black, black, current_route == L1 ? green : white);
    btn_l2.render(black, black, current_route == L2 ? green : white);
    btn_l3.render(black, black, current_route == L3 ? green : white);
#endif
#ifdef RIGHT
    btn_r1.render(black, black, current_route == R1 ? green : white);
    btn_r2.render(black, black, current_route == R2 ? green : white);
    btn_r3.render(black, black, current_route == R3 ? green : white);
#endif
    btn_skills.render(black, black, current_route == SKILLS ? green : white);

    lbl_color.render();
    btn_red.render(black, black, current_color == RED ? red : white);
    btn_blue.render(black, black, current_color == BLUE ? blue : white);

    lbl_mode.render();
    btn_competition.render(black, black, current_mode == COMPETITION ? green : white);
    btn_pid_debug.render(black, black, current_mode == PID_DEBUG ? green : white);
    btn_route_debug.render(black, black, current_mode == ROUTE_DEBUG ? green : white);
}

void fuckMotor(motor to_be_fucked, std::string name, int row) {
    Brain.Screen.setPenColor(to_be_fucked.installed() ? black : red);
    Brain.Screen.setFont(mono15);
    Brain.Screen.setCursor(row + 4, 1);
    Brain.Screen.print("%s pos: %.1f, temp: %.1f       ", name.c_str(),
                       to_be_fucked.position(deg),
                       to_be_fucked.temperature(temperatureUnits::celsius));
}

void displayMotors() {
    Brain.Screen.setFillColor(white);
    Brain.Screen.setFont(mono15);
#ifdef XDRIVE
    fuckMotor(Motor_Base_LFU, "Motor_Base_LFU", 0);
    fuckMotor(Motor_Base_LFD, "Motor_Base_LFD", 1);
    fuckMotor(Motor_Base_LBU, "Motor_Base_LBU", 2);
    fuckMotor(Motor_Base_LBD, "Motor_Base_LBD", 3);
    fuckMotor(Motor_Base_RBU, "Motor_Base_RBU", 4);
    fuckMotor(Motor_Base_RBD, "Motor_Base_RBD", 5);
    fuckMotor(Motor_Base_RFU, "Motor_Base_RFU", 6);
    fuckMotor(Motor_Base_RFD, "Motor_Base_RFD", 7);
#endif
#ifdef STRAIGHT
    fuckMotor(Motor_Base_L1, "[Motor_Base_L1]", 0);
    fuckMotor(Motor_Base_L2, "[Motor_Base_L2]", 1);
    fuckMotor(Motor_Base_L3, "[Motor_Base_L3]", 2);
    fuckMotor(Motor_Base_L4, "[Motor_Base_L4]", 3);
    fuckMotor(Motor_Base_R1, "[Motor_Base_R1]", 4);
    fuckMotor(Motor_Base_R2, "[Motor_Base_R2]", 5);
    fuckMotor(Motor_Base_R3, "[Motor_Base_R3]", 6);
    fuckMotor(Motor_Base_R4, "[Motor_Base_R4]", 7);
#endif
    fuckMotor(Motor_Intaker1, "[Motors_Intaker1]", 8);
    fuckMotor(Motor_Intaker2, "[Motors_Intaker2]", 9);
    fuckMotor(Motor_Roller, "[Motor_Roller] ", 10);
    fuckMotor(Motor_Shooter, "[Motor_Shooter]", 11);
    Brain.Screen.setFont(mono20);
}

void displayDevices() {
    Brain.Screen.setFillColor(white);

    Brain.Screen.setPenColor(Inertial.installed() ? black : red);
    Brain.Screen.printAt(0, 3 * 20, "[Inertial] heading: %.2f        ",
                         Inertial.heading(deg));
#ifdef POSITION_ORTHOGONAL
    Brain.Screen.setPenColor(Rotation_L.installed() ? black : red);
    Brain.Screen.printAt(0, 4 * 20, "[Rotation_L] deg: %.2f        ",
                         Rotation_L.position(deg));
    Brain.Screen.setPenColor(Rotation_R.installed() ? black : red);
    Brain.Screen.printAt(0, 5 * 20, "[Rotation_R] deg: %.2f        ",
                         Rotation_R.position(deg));
#endif
#ifdef POSITION_DIFFERENTIAL
    Brain.Screen.setPenColor(Rotation.installed() ? black : red);
    Brain.Screen.printAt(0, 4 * 20, "[Rotation] deg: %.2f        ",
                         Rotation.position(deg));
#endif
    Brain.Screen.setPenColor(Optical_Exit.installed() ? black : red);
    Brain.Screen.printAt(0, 6 * 20, "[Optical_Exit] val: %d", Optical_Exit.value());

    Brain.Screen.setPenColor(Distance_Entrance.installed() ? black : red);
    Brain.Screen.printAt(0, 7 * 20, "[Distance_Entrance] dist: %d",
                         Distance_Entrance.objectDistance(mm));
    Brain.Screen.setPenColor(Distance_Exit.installed() ? black : red);
    Brain.Screen.printAt(0, 8 * 20, "[Distance_Exit] dist: %d",
                         Distance_Exit.objectDistance(mm));

    Brain.Screen.setPenColor(black);
    Brain.Screen.printAt(0, 9 * 20, "[*Piston_Load] state: %d", Piston_Load.value());
    Brain.Screen.printAt(0, 10 * 20, "[*Piston_Shooter] state: %d",
                         Piston_Shooter.value());
    Brain.Screen.printAt(0, 11 * 20, "[*Piston_Hook] state: %d", Piston_Hook.value());
}

void displayInfo() {
    Brain.Screen.setPenColor(black);
    Brain.Screen.setFillColor(white);
    const Point pos = myPosition.getCenterPos();
    Brain.Screen.printAt(0, 3 * 20, "[Position] X: %.2f, Y: %.2f, H: %.2f        ",
                         pos.x_, pos.y_, myPosition.getHeading());
    Brain.Screen.printAt(0, 4 * 20, "[Velocity] Vx: %.2f, Vy: %.2f, Vh: %.2f        ",
                         myPosition.getTransVel().dx_, myPosition.getTransVel().dy_,
                         myPosition.getTransVel().angle());
}

void displayParams() {
#ifdef XDRIVE
    lbl_trans.render();
    lbl_rot.render();
    lbl_back.render();
    lbl_kp.render();
    lbl_ki.render();
    lbl_kd.render();

    lbl_trans_kp.setText(config.trace.PID_trans.kP_);
    lbl_trans_ki.setText(config.trace.PID_trans.kI_);
    lbl_trans_kd.setText(config.trace.PID_trans.kD_);
    lbl_rot_kp.setText(config.trace.PID_rot.kP_);
    lbl_rot_ki.setText(config.trace.PID_rot.kI_);
    lbl_rot_kd.setText(config.trace.PID_rot.kD_);
    lbl_back_kp.setText(config.trace.PID_back.kP_);
    lbl_back_ki.setText(config.trace.PID_back.kI_);
    lbl_back_kd.setText(config.trace.PID_back.kD_);

    lbl_trans_kp.render();
    lbl_trans_ki.render();
    lbl_trans_kd.render();
    lbl_rot_kp.render();
    lbl_rot_ki.render();
    lbl_rot_kd.render();
    lbl_back_kp.render();
    lbl_back_ki.render();
    lbl_back_kd.render();

    btn_trans_kp_add.render(black, black, blue);
    btn_trans_ki_add.render(black, black, blue);
    btn_trans_kd_add.render(black, black, blue);
    btn_rot_kp_add.render(black, black, blue);
    btn_rot_ki_add.render(black, black, blue);
    btn_rot_kd_add.render(black, black, blue);
    btn_back_kp_add.render(black, black, blue);
    btn_back_ki_add.render(black, black, blue);
    btn_back_kd_add.render(black, black, blue);

    btn_trans_kp_minus.render(black, black, red);
    btn_trans_ki_minus.render(black, black, red);
    btn_trans_kd_minus.render(black, black, red);
    btn_rot_kp_minus.render(black, black, red);
    btn_rot_ki_minus.render(black, black, red);
    btn_rot_kd_minus.render(black, black, red);
    btn_back_kp_minus.render(black, black, red);
    btn_back_ki_minus.render(black, black, red);
    btn_back_kd_minus.render(black, black, red);
#endif
#ifdef STRAIGHT
    btn_profile_fwd.render(black, black,
                           current_param_profile == PROFILE_FWD ? green : white);
    btn_profile_turn.render(black, black,
                            current_param_profile == PROFILE_TURN ? green : white);
    btn_profile_pose_far.render(
        black, black, current_param_profile == PROFILE_POSE_FAR ? green : white);
    btn_profile_pose_bad.render(
        black, black, current_param_profile == PROFILE_POSE_CLOSE_BAD ? green : white);
    btn_profile_pose_good.render(
        black, black, current_param_profile == PROFILE_POSE_CLOSE_GOOD ? green : white);

    lbl_fb.render();
    lbl_rot.render();
    lbl_kp.render();
    lbl_ki.render();
    lbl_kd.render();

    const PIDParam* fb_pid = selectedFbPid();
    const PIDParam* rot_pid = selectedRotPid();
    lbl_fb_kp.setText(fb_pid->kP_);
    lbl_fb_ki.setText(fb_pid->kI_);
    lbl_fb_kd.setText(fb_pid->kD_);
    lbl_rot_kp.setText(rot_pid->kP_);
    lbl_rot_ki.setText(rot_pid->kI_);
    lbl_rot_kd.setText(rot_pid->kD_);

    lbl_fb_kp.render();
    lbl_fb_ki.render();
    lbl_fb_kd.render();
    lbl_rot_kp.render();
    lbl_rot_ki.render();
    lbl_rot_kd.render();

    btn_fb_kp_add.render(black, black, blue);
    btn_fb_ki_add.render(black, black, blue);
    btn_fb_kd_add.render(black, black, blue);
    btn_rot_kp_add.render(black, black, blue);
    btn_rot_ki_add.render(black, black, blue);
    btn_rot_kd_add.render(black, black, blue);

    btn_fb_kp_minus.render(black, black, red);
    btn_fb_ki_minus.render(black, black, red);
    btn_fb_kd_minus.render(black, black, red);
    btn_rot_kp_minus.render(black, black, red);
    btn_rot_ki_minus.render(black, black, red);
    btn_rot_kd_minus.render(black, black, red);
#endif
}

void updateControllerUI() {
    if (!initializing) {
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("L: %.2f R: %.2f     ", myPosition.getLeftDist(),
                                myPosition.getRightDist());

        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print(
            "X: %.1f Y: %.1f H: %.2f        ", myPosition.getCenterPos().x_,
            myPosition.getCenterPos().y_, myPosition.getHeadingUnwrapped());

        Controller.Screen.setCursor(3, 5);
        Controller.Screen.print("%s                 ",
                                auto_route_name[static_cast<int>(current_route)].c_str());
        Controller.Screen.setCursor(3, 1);
        Controller.Screen.print(current_color == BLUE ? "BLUE" : "RED ");
    }
}

void updateBrainUI() {
    if (current_interface != OPTIONS) btn_options.check();
    if (current_interface != MOTORS) btn_motors.check();
    if (current_interface != DEVICES) btn_devices.check();
    if (current_interface != INFO) btn_info.check();
    if (current_interface != PARAMS && current_mode == PID_DEBUG) btn_params.check();
    switch (current_interface) {
        case OPTIONS:
#ifdef LEFT
            if (current_route != L1) btn_l1.check();
            if (current_route != L2) btn_l2.check();
            if (current_route != L3) btn_l3.check();
#endif
#ifdef RIGHT
            if (current_route != R1) btn_r1.check();
            if (current_route != R2) btn_r2.check();
            if (current_route != R3) btn_r3.check();
#endif
            if (current_route != SKILLS) btn_skills.check();

            if (current_color != RED) btn_red.check();
            if (current_color != BLUE) btn_blue.check();

            if (current_mode != COMPETITION) btn_competition.check();
            if (current_mode != ROUTE_DEBUG) btn_route_debug.check();
            if (current_mode != PID_DEBUG) btn_pid_debug.check();
            break;
        case MOTORS:
            displayMotors();
            break;
        case DEVICES:
            displayDevices();
            break;
        case INFO:
            displayInfo();
            break;
        case PARAMS:
#ifdef XDRIVE
            btn_trans_kp_add.check();
            btn_trans_ki_add.check();
            btn_trans_kd_add.check();
            btn_rot_kp_add.check();
            btn_rot_ki_add.check();
            btn_rot_kd_add.check();
            btn_back_kp_add.check();
            btn_back_ki_add.check();
            btn_back_kd_add.check();

            btn_trans_kp_minus.check();
            btn_trans_ki_minus.check();
            btn_trans_kd_minus.check();
            btn_rot_kp_minus.check();
            btn_rot_ki_minus.check();
            btn_rot_kd_minus.check();
            btn_back_kp_minus.check();
            btn_back_ki_minus.check();
            btn_back_kd_minus.check();
#endif
#ifdef STRAIGHT
            if (current_param_profile != PROFILE_FWD) btn_profile_fwd.check();
            if (current_param_profile != PROFILE_TURN) btn_profile_turn.check();
            if (current_param_profile != PROFILE_POSE_FAR) btn_profile_pose_far.check();
            if (current_param_profile != PROFILE_POSE_CLOSE_BAD)
                btn_profile_pose_bad.check();
            if (current_param_profile != PROFILE_POSE_CLOSE_GOOD)
                btn_profile_pose_good.check();

            btn_fb_kp_add.check();
            btn_fb_ki_add.check();
            btn_fb_kd_add.check();
            btn_rot_kp_add.check();
            btn_rot_ki_add.check();
            btn_rot_kd_add.check();
            btn_fb_kp_minus.check();
            btn_fb_ki_minus.check();
            btn_fb_kd_minus.check();
            btn_rot_kp_minus.check();
            btn_rot_ki_minus.check();
            btn_rot_kd_minus.check();
#endif
            break;
    }
}

void updateUI() {
    clearBrain();
    clearController();
    displayTopBar();
    switch (current_interface) {
        case OPTIONS:
            displayOptions();
            break;
        case MOTORS:
            displayMotors();
            break;
        case DEVICES:
            displayDevices();
            break;
        case INFO:
            displayInfo();
            break;
        case PARAMS:
            displayParams();
            break;
    }
    while (true) {
        updateBrainUI();
        updateControllerUI();
        this_thread::sleep_for(cycle.ui);
    }
}
