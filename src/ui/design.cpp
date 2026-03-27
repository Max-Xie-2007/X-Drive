#include "ui/design.h"
#include "utils/pid.h"
#include <algorithm>
#include <cmath>

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
    fuckMotor(Motor_Base_LF, "Motor_Base_LF", 0);
    fuckMotor(Motor_Base_LB, "Motor_Base_LB", 1);
    fuckMotor(Motor_Base_RF, "Motor_Base_RF", 2);
    fuckMotor(Motor_Base_RB, "Motor_Base_RB", 3);
    Brain.Screen.setFont(mono20);
}

void displayDevices() {
    Brain.Screen.setFillColor(white);

    Brain.Screen.setPenColor(Inertial.installed() ? black : red);
    Brain.Screen.printAt(0, 3 * 20, "[Inertial] heading: %.2f        ",
                         Inertial.heading(deg));
    Brain.Screen.setPenColor(Rotation_L.installed() ? black : red);
    Brain.Screen.printAt(0, 4 * 20, "[Rotation_L] deg: %.2f        ",
                         Rotation_L.position(deg));
    Brain.Screen.setPenColor(Rotation_R.installed() ? black : red);
    Brain.Screen.printAt(0, 5 * 20, "[Rotation_R] deg: %.2f        ",
                         Rotation_R.position(deg));
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
