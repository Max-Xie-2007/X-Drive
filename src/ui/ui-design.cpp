#include "ui/ui-design.h"
#include "globals.h"
#include "utils/pid.h"
#include <algorithm>
#include <cmath>
#include <cstdio>

static PIDParam* selectedTranslationalPid() {
    switch (current_param_profile) {
        case PROFILE_REACH:
            return &default_reach_translational;
        case PROFILE_TRACE:
            return &default_trace_translational;
    }
    return &default_reach_translational;
}

static PIDParam* selectedAngularPid() {
    switch (current_param_profile) {
        case PROFILE_REACH:
            return &default_reach_angular;
        case PROFILE_TRACE:
            return &default_trace_angular;
    }
    return &default_reach_angular;
}

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
Button btn_r1(20, 90, 60, 60, "R1", [] { setRoute(R1); });
Button btn_r2(90, 90, 60, 60, "R2", [] { setRoute(R2); });
Button btn_r3(160, 90, 60, 60, "R3", [] { setRoute(R3); });
#endif
Button btn_skills(20, 160, 200, 60, "SKILLS", [] { setRoute(SKILLS); });

Button btn_red(260, 90, 60, 60, "RED", [] { setColor(RED); });
Button btn_blue(260, 160, 60, 60, "BLUE", [] { setColor(BLUE); });

Button btn_competition(360, 85, 100, 40, "COMP", [] { setMode(COMPETITION); });
Button btn_pid_debug(360, 135, 100, 40, "PID", [] { setMode(PID_DEBUG); });
Button btn_route_debug(360, 185, 100, 40, "ROUTE", [] { setMode(ROUTE_DEBUG); });

Button btn_profile_reach(0, 45, 96, 35, "FWD", [] { setParamProfile(PROFILE_REACH); });
Button btn_profile_trace(96, 45, 96, 35, "TURN", [] { setParamProfile(PROFILE_TRACE); });

Label lbl_translational(0, 130, 30, 60, "LAT");
Label lbl_angular(0, 185, 30, 60, "ANG");
Label lbl_kp(30, 95, 150, 35, "kP");
Label lbl_ki(180, 95, 150, 35, "kI");
Label lbl_kd(330, 95, 150, 35, "kD");

Label lbl_translational_kp(80, 130, 50, 55, default_reach_translational.kP_);
Label lbl_translational_ki(230, 130, 50, 55, default_reach_translational.kI_);
Label lbl_translational_kd(380, 130, 50, 55, default_reach_translational.kD_);
Label lbl_angular_kp(80, 185, 50, 55, default_reach_angular.kP_);
Label lbl_angular_ki(230, 185, 50, 55, default_reach_angular.kI_);
Label lbl_angular_kd(380, 185, 50, 55, default_reach_angular.kD_);

const double translational_kp_step = 0.1;
const double translational_ki_step = 0.001;
const double translational_kd_step = 1.0;
const double angular_kp_step = 0.02;
const double angular_ki_step = 0.001;
const double angular_kd_step = 1.0;

Button btn_translational_kp_add(130, 130, 50, 55, "+", [] {
    selectedTranslationalPid()->kP_ += translational_kp_step;
    refresh();
});
Button btn_translational_ki_add(280, 130, 50, 55, "+", [] {
    selectedTranslationalPid()->kI_ += translational_ki_step;
    refresh();
});
Button btn_translational_kd_add(430, 130, 50, 55, "+", [] {
    selectedTranslationalPid()->kD_ += translational_kd_step;
    refresh();
});
Button btn_translational_kp_minus(30, 130, 50, 55, "-", [] {
    selectedTranslationalPid()->kP_ -= translational_kp_step;
    refresh();
});
Button btn_translational_ki_minus(180, 130, 50, 55, "-", [] {
    selectedTranslationalPid()->kI_ -= translational_ki_step;
    refresh();
});
Button btn_translational_kd_minus(330, 130, 50, 55, "-", [] {
    selectedTranslationalPid()->kD_ -= translational_kd_step;
    refresh();
});
Button btn_angular_kp_add(130, 185, 50, 55, "+", [] {
    selectedAngularPid()->kP_ += angular_kp_step;
    refresh();
});
Button btn_angular_ki_add(280, 185, 50, 55, "+", [] {
    selectedAngularPid()->kI_ += angular_ki_step;
    refresh();
});
Button btn_angular_kd_add(430, 185, 50, 55, "+", [] {
    selectedAngularPid()->kD_ += angular_kd_step;
    refresh();
});
Button btn_angular_kp_minus(30, 185, 50, 55, "-", [] {
    selectedAngularPid()->kP_ -= angular_kp_step;
    refresh();
});
Button btn_angular_ki_minus(180, 185, 50, 55, "-", [] {
    selectedAngularPid()->kI_ -= angular_ki_step;
    refresh();
});
Button btn_angular_kd_minus(330, 185, 50, 55, "-", [] {
    selectedAngularPid()->kD_ -= angular_kd_step;
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

void printMotorInfo(motor the_motor, std::string name, int row) {
    Brain.Screen.setPenColor(the_motor.installed() ? black : red);
    Brain.Screen.setFont(mono15);
    Brain.Screen.setCursor(row + 4, 1);
    Brain.Screen.print("%s pos: %.1f, temp: %.1f       ", name.c_str(),
                       the_motor.position(deg),
                       the_motor.temperature(temperatureUnits::celsius));
}

void displayMotors() {
    Brain.Screen.setFillColor(white);
    Brain.Screen.setFont(mono15);
    for (int i = 0; i < base_motor_count; i++) {
        char motor_name[20];
        snprintf(motor_name, sizeof(motor_name), "[Motor_Base_L%d]", i + 1);
        printMotorInfo(Motors_Base_LF[i], motor_name, i);
    }
    for (int i = 0; i < base_motor_count; i++) {
        char motor_name[20];
        snprintf(motor_name, sizeof(motor_name), "[Motor_Base_L%d]", i + 1);
        printMotorInfo(Motors_Base_LB[i], motor_name, i);
    }
    for (int i = 0; i < base_motor_count; i++) {
        char motor_name[20];
        snprintf(motor_name, sizeof(motor_name), "[Motor_Base_L%d]", i + 1);
        printMotorInfo(Motors_Base_RF[i], motor_name, i);
    }
    for (int i = 0; i < base_motor_count; i++) {
        char motor_name[20];
        snprintf(motor_name, sizeof(motor_name), "[Motor_Base_R%d]", i + 1);
        printMotorInfo(Motors_Base_RB[i], motor_name, i + base_motor_count);
    }
    Brain.Screen.setFont(mono20);
}

void displayDevices() {
    Brain.Screen.setFont(mono20);

    Brain.Screen.setFillColor(white);

    Brain.Screen.setCursor(4, 1);
    Brain.Screen.setPenColor(Inertial.installed() ? black : red);
    Brain.Screen.print("[Inertial] heading: %.2f        ", Inertial.heading(deg));
}

void displayInfo() {
    Brain.Screen.setPenColor(black);
    Brain.Screen.setFillColor(white);
    const Point pos = myPosition.getCenterPos();
    Brain.Screen.printAt(0, 3 * 20, "[Position] X: %.2f, Y: %.2f, H: %.2f        ",
                         pos.x_, pos.y_, myPosition.getHeading());
    Brain.Screen.printAt(0, 4 * 20, "[Velocity] Vx: %.2f, Vy: %.2f, Vh: %.2f        ",
                         myPosition.getTranslationalVelocity().dx_,
                         myPosition.getTranslationalVelocity().dy_,
                         myPosition.getTranslationalVelocity().angle());
}

void displayParams() {
    btn_profile_reach.render(black, black,
                             current_param_profile == PROFILE_REACH ? green : white);
    btn_profile_trace.render(
        black, black, current_param_profile == PROFILE_TRACE ? green : white);

    lbl_translational.render();
    lbl_angular.render();
    lbl_kp.render();
    lbl_ki.render();
    lbl_kd.render();

    const PIDParam* lateral_pid = selectedTranslationalPid();
    const PIDParam* angular_pid = selectedAngularPid();
    lbl_translational_kp.setText(lateral_pid->kP_);
    lbl_translational_ki.setText(lateral_pid->kI_);
    lbl_translational_kd.setText(lateral_pid->kD_);
    lbl_angular_kp.setText(angular_pid->kP_);
    lbl_angular_ki.setText(angular_pid->kI_);
    lbl_angular_kd.setText(angular_pid->kD_);

    lbl_translational_kp.render();
    lbl_translational_ki.render();
    lbl_translational_kd.render();
    lbl_angular_kp.render();
    lbl_angular_ki.render();
    lbl_angular_kd.render();

    btn_translational_kp_add.render(black, black, blue);
    btn_translational_ki_add.render(black, black, blue);
    btn_translational_kd_add.render(black, black, blue);
    btn_angular_kp_add.render(black, black, blue);
    btn_angular_ki_add.render(black, black, blue);
    btn_angular_kd_add.render(black, black, blue);

    btn_translational_kp_minus.render(black, black, red);
    btn_translational_ki_minus.render(black, black, red);
    btn_translational_kd_minus.render(black, black, red);
    btn_angular_kp_minus.render(black, black, red);
    btn_angular_ki_minus.render(black, black, red);
    btn_angular_kd_minus.render(black, black, red);
}

void updateControllerUI() {
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("L %.1f R %.1f             ", myPosition.getLeftDist(),
                            myPosition.getRightDist());

    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("X %.1f Y %.1f H %.2f        ", myPosition.getCenterPos().x_,
                            myPosition.getCenterPos().y_,
                            myPosition.getHeadingUnwrapped());

    Controller.Screen.setCursor(3, 5);
    Controller.Screen.print("%s                 ",
                            auto_route_name[static_cast<int>(current_route)].c_str());
    Controller.Screen.setCursor(3, 1);
    Controller.Screen.print(current_color == BLUE ? "BLUE" : "RED ");
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
            if (current_param_profile != PROFILE_REACH) btn_profile_reach.check();
            if (current_param_profile != PROFILE_TRACE) btn_profile_trace.check();

            btn_translational_kp_add.check();
            btn_translational_ki_add.check();
            btn_translational_kd_add.check();
            btn_angular_kp_add.check();
            btn_angular_ki_add.check();
            btn_angular_kd_add.check();
            btn_translational_kp_minus.check();
            btn_translational_ki_minus.check();
            btn_translational_kd_minus.check();
            btn_angular_kp_minus.check();
            btn_angular_ki_minus.check();
            btn_angular_kd_minus.check();
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
        if (!initializing) {
            updateControllerUI();
        }
        this_thread::sleep_for(cycle.ui_update);
    }
}
