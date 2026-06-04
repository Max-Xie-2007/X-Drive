#include "ui/ui-brain.h"
#include "globals.h"
#include "utils/pid.h"

Interface current_interface = Interface::OPTIONS;
PIDParamSet current_param_set = PIDParamSet::REACH45;

void setInterface(Interface interface) {
    if (current_interface == interface) return;
    current_interface = interface;
    clearBrain();
    displayTopBar();
    switch (current_interface) {
        case Interface::OPTIONS:
            displayOptions();
            break;
        case Interface::AUTON:
            displayAuton();
            break;
        case Interface::MOTORS:
            displayMotors();
            break;
        case Interface::DEVICES:
            displayDevices();
            break;
        case Interface::INFO:
            displayInfo();
            break;
        case Interface::PARAMS:
            displayParams();
            break;
    }
}

// top bar buttons

Button btn_options(0, 0, 80, 40, "Options", [] { setInterface(Interface::OPTIONS); });
Button btn_auton(80, 0, 80, 40, "Auton", [] { setInterface(Interface::AUTON); });
Button btn_motors(160, 0, 80, 40, "Motors", [] { setInterface(Interface::MOTORS); });
Button btn_devices(240, 0, 80, 40, "Devices", [] { setInterface(Interface::DEVICES); });
Button btn_info(320, 0, 80, 40, "Info", [] { setInterface(Interface::INFO); });
Button btn_params(400, 0, 80, 40, "Params", [] { setInterface(Interface::PARAMS); });

// options interface labels and buttons

Label lbl_drivers(20, 50, 220, 30, "Drivers");
Button btn_chifeng(20, 90, 90, 60, "CHIFENG", [] { setDriver(Driver::CHIFENG); });
Button btn_xtr(130, 90, 90, 60, "XTR", [] { setDriver(Driver::XTR); });
Button btn_cks(20, 160, 90, 60, "CKS", [] { setDriver(Driver::CKS); });
Button btn_zyx(130, 160, 90, 60, "ZYX", [] { setDriver(Driver::ZYX); });

Label lbl_color(240, 50, 60, 30, "Color");
Button btn_red(240, 90, 60, 60, "RED", [] {
    setColor(current_color != Color::RED ? Color::RED : Color::NEITHER);
});
Button btn_blue(240, 160, 60, 60, "BLUE", [] {
    setColor(current_color != Color::BLUE ? Color::BLUE : Color::NEITHER);
});

Label lbl_mode(320, 50, 140, 30, "Mode");

Button btn_competition(320, 90, 65, 60, "COMP", [] { setMode(Mode::COMPETITION); });
Button btn_practice(320, 160, 65, 60, "PRAC", [] { setMode(Mode::PRACTICE); });
Button btn_pid_debug(395, 90, 65, 60, "PID", [] { setMode(Mode::PID_DEBUG); });
Button btn_route_debug(395, 160, 65, 60, "ROUTE", [] { setMode(Mode::ROUTE_DEBUG); });

// routes interface labels and buttons

Label lbl_tournament(0, 50, 240, 30, "Tournament");
Button btn_tour_1(20, 90, 60, 60, "1", [] { setRoute(Route::TOUR1); });
Button btn_tour_2(90, 90, 60, 60, "2", [] { setRoute(Route::TOUR2); });
Button btn_tour_3(160, 90, 60, 60, "3", [] { setRoute(Route::TOUR3); });
Button btn_tour_D(20, 160, 200, 60, "Dark Fenix", [] { setRoute(Route::TOURD); });

Label lbl_skills(240, 50, 140, 30, "Skills");
Button btn_skills_solo(260, 160, 100, 60, "SOLO", [] { setRoute(Route::SKILLS_SOLO); });
Button btn_skills_coop(260, 90, 100, 60, "COOP", [] { setRoute(Route::SKILLS_COOP); });

Label lbl_side(380, 50, 100, 30, "Side");
Button btn_left(400, 90, 60, 60, "LEFT", [] { setSide(Side::LEFT); });
Button btn_right(400, 160, 60, 60, "RIGHT", [] { setSide(Side::RIGHT); });

// params interface labels and buttons

Label lbl_translational(0, 129, 30, 37, "TRN");
Label lbl_angular(0, 166, 30, 37, "ANG");
Label lbl_deviational(0, 203, 30, 37, "DEV");
Label lbl_kp(30, 95, 150, 35, "kP");
Label lbl_ki(180, 95, 150, 35, "kI");
Label lbl_kd(330, 95, 150, 35, "kD");

Label lbl_translational_kp(80, 129, 50, 37, default_reach_translational.kP_);
Label lbl_translational_ki(230, 129, 50, 37, default_reach_translational.kI_);
Label lbl_translational_kd(380, 129, 50, 37, default_reach_translational.kD_);
Label lbl_angular_kp(80, 166, 50, 37, default_reach_angular_90.kP_);
Label lbl_angular_ki(230, 166, 50, 37, default_reach_angular_90.kI_);
Label lbl_angular_kd(380, 166, 50, 37, default_reach_angular_90.kD_);
Label lbl_deviational_kp(80, 203, 50, 37, default_trace_deviational.kP_);
Label lbl_deviational_ki(230, 203, 50, 37, default_trace_deviational.kI_);
Label lbl_deviational_kd(380, 203, 50, 37, default_trace_deviational.kD_);

const float translational_kp_step = 0.05;
const float translational_ki_step = 1.0;
const float translational_kd_step = 0.01;
const float angular_kp_step = 0.02;
const float angular_ki_step = 1.0;
const float angular_kd_step = 0.005;

static PIDParam* selectedTranslationalPid() {
    switch (current_param_set) {
        case PIDParamSet::REACH45:
        case PIDParamSet::REACH90:
        case PIDParamSet::REACH135:
            return &default_reach_translational;
        case PIDParamSet::TRACE:
            return &default_trace_translational;
    }
}

static PIDParam* selectedAngularPid() {
    switch (current_param_set) {
        case PIDParamSet::REACH45:
            return &default_reach_angular_45;
        case PIDParamSet::REACH90:
            return &default_reach_angular_90;
        case PIDParamSet::REACH135:
            return &default_reach_angular_135;
        case PIDParamSet::TRACE:
            return &default_trace_angular;
    }
}

static PIDParam* selectedDeviationalPid() {
    switch (current_param_set) {
        case PIDParamSet::REACH45:
        case PIDParamSet::REACH90:
        case PIDParamSet::REACH135:
            return nullptr;
        case PIDParamSet::TRACE:
            return &default_trace_deviational;
    }
}

Button btn_param_set_reach_45(0, 40, 120, 50, "REACH45", [] {
    current_param_set = PIDParamSet::REACH45;
    displayParams();
});
Button btn_param_set_reach_90(120, 40, 120, 50, "REACH90", [] {
    current_param_set = PIDParamSet::REACH90;
    displayParams();
});
Button btn_param_set_reach_135(240, 40, 120, 50, "REACH135", [] {
    current_param_set = PIDParamSet::REACH135;
    displayParams();
});
Button btn_param_set_trace(360, 40, 120, 50, "TRACE", [] {
    current_param_set = PIDParamSet::TRACE;
    displayParams();
});

Button btn_translational_kp_add(130, 129, 50, 37, "+", [] {
    selectedTranslationalPid()->kP_ += translational_kp_step;
    displayParams();
});
Button btn_translational_ki_add(280, 129, 50, 37, "+", [] {
    selectedTranslationalPid()->kI_ += translational_ki_step;
    displayParams();
});
Button btn_translational_kd_add(430, 129, 50, 37, "+", [] {
    selectedTranslationalPid()->kD_ += translational_kd_step;
    displayParams();
});
Button btn_translational_kp_minus(30, 129, 50, 37, "-", [] {
    selectedTranslationalPid()->kP_ -= translational_kp_step;
    displayParams();
});
Button btn_translational_ki_minus(180, 129, 50, 37, "-", [] {
    selectedTranslationalPid()->kI_ -= translational_ki_step;
    displayParams();
});
Button btn_translational_kd_minus(330, 129, 50, 37, "-", [] {
    selectedTranslationalPid()->kD_ -= translational_kd_step;
    displayParams();
});
Button btn_angular_kp_add(130, 166, 50, 37, "+", [] {
    selectedAngularPid()->kP_ += angular_kp_step;
    displayParams();
});
Button btn_angular_ki_add(280, 166, 50, 37, "+", [] {
    selectedAngularPid()->kI_ += angular_ki_step;
    displayParams();
});
Button btn_angular_kd_add(430, 166, 50, 37, "+", [] {
    selectedAngularPid()->kD_ += angular_kd_step;
    displayParams();
});
Button btn_angular_kp_minus(30, 166, 50, 37, "-", [] {
    selectedAngularPid()->kP_ -= angular_kp_step;
    displayParams();
});
Button btn_angular_ki_minus(180, 166, 50, 37, "-", [] {
    selectedAngularPid()->kI_ -= angular_ki_step;
    displayParams();
});
Button btn_angular_kd_minus(330, 166, 50, 37, "-", [] {
    selectedAngularPid()->kD_ -= angular_kd_step;
    displayParams();
});
Button btn_deviational_kp_add(130, 203, 50, 37, "+", [] {
    if (selectedDeviationalPid() == nullptr) return;
    selectedDeviationalPid()->kP_ += angular_kp_step;
    displayParams();
});
Button btn_deviational_ki_add(280, 203, 50, 37, "+", [] {
    if (selectedDeviationalPid() == nullptr) return;
    selectedDeviationalPid()->kI_ += angular_ki_step;
    displayParams();
});
Button btn_deviational_kd_add(430, 203, 50, 37, "+", [] {
    if (selectedDeviationalPid() == nullptr) return;
    selectedDeviationalPid()->kD_ += angular_kd_step;
    displayParams();
});
Button btn_deviational_kp_minus(30, 203, 50, 37, "-", [] {
    if (selectedDeviationalPid() == nullptr) return;
    selectedDeviationalPid()->kP_ -= angular_kp_step;
    displayParams();
});
Button btn_deviational_ki_minus(180, 203, 50, 37, "-", [] {
    if (selectedDeviationalPid() == nullptr) return;
    selectedDeviationalPid()->kI_ -= angular_ki_step;
    displayParams();
});
Button btn_deviational_kd_minus(330, 203, 50, 37, "-", [] {
    if (selectedDeviationalPid() == nullptr) return;
    selectedDeviationalPid()->kD_ -= angular_kd_step;
    displayParams();
});

void clearBrain() { Brain.Screen.clearScreen(white); }

void displayTopBar() {
    btn_options.render(black, black,
                       current_interface == Interface::OPTIONS ? green : white);
    btn_auton.render(black, black, current_interface == Interface::AUTON ? green : white);
    btn_motors.render(black, black,
                      current_interface == Interface::MOTORS ? green : white);
    btn_devices.render(black, black,
                       current_interface == Interface::DEVICES ? green : white);
    btn_info.render(black, black, current_interface == Interface::INFO ? green : white);
    if (current_interface == Interface::PARAMS) {
        btn_params.render(black, black, green);
    } else if (current_mode == Mode::PID_DEBUG) {
        btn_params.render(black, black, white);
    } else {
        btn_params.render(black, black, color(192, 192, 192));
    }
}

void displayOptions() {
    lbl_drivers.render(black, white, white);
    btn_chifeng.render(black, black, current_driver == Driver::CHIFENG ? green : white);
    btn_xtr.render(black, black, current_driver == Driver::XTR ? green : white);
    btn_cks.render(black, black, current_driver == Driver::CKS ? green : white);
    btn_zyx.render(black, black, current_driver == Driver::ZYX ? green : white);

    lbl_color.render(black, white, white);
    if (current_color == Color::RED) {
        btn_red.render(black, black, red);
        btn_blue.render(black, black, white);
    } else if (current_color == Color::BLUE) {
        btn_blue.render(black, black, blue);
        btn_red.render(black, black, white);
    } else {
        btn_red.render(black, black, white);
        btn_blue.render(black, black, white);
    }

    lbl_mode.render(black, white, white);
    btn_competition.render(black, black,
                           current_mode == Mode::COMPETITION ? green : white);
    btn_practice.render(black, black, current_mode == Mode::PRACTICE ? green : white);
    btn_pid_debug.render(black, black, current_mode == Mode::PID_DEBUG ? green : white);
    btn_route_debug.render(black, black,
                           current_mode == Mode::ROUTE_DEBUG ? green : white);
}

void displayAuton() {
    lbl_tournament.render(black, white, white);
    btn_tour_1.render(black, black, current_route == Route::TOUR1 ? green : white);
    btn_tour_2.render(black, black, current_route == Route::TOUR2 ? green : white);
    btn_tour_3.render(black, black, current_route == Route::TOUR3 ? green : white);
    btn_tour_D.render(black, black, current_route == Route::TOURD ? green : white);

    lbl_skills.render(black, white, white);
    btn_skills_coop.render(black, black,
                           current_route == Route::SKILLS_COOP ? green : white);
    btn_skills_solo.render(black, black,
                           current_route == Route::SKILLS_SOLO ? green : white);

    lbl_side.render(black, white, white);
    btn_left.render(black, black, current_side == Side::LEFT ? green : white);
    btn_right.render(black, black, current_side == Side::RIGHT ? green : white);
}

void printMotorInfo(motor& the_motor, std::string name, int port, int row) {
    Brain.Screen.setPenColor(the_motor.installed() ? black : red);
    Brain.Screen.setFont(mono15);
    Brain.Screen.setCursor(row + 3, 1);
    Brain.Screen.print("%s pos: %.1f temp: %.1f port: %d       ", name.c_str(),
                       the_motor.position(deg),
                       the_motor.temperature(temperatureUnits::celsius), port);
}
void displayMotors() {
    Brain.Screen.setFont(mono15);
    Brain.Screen.setFillColor(white);
    printMotorInfo(Motors_Base_LF[0], "[Motor_Base_LF]", 11, 1);
    printMotorInfo(Motors_Base_LB[0], "[Motor_Base_LB]", 12, 2);
    printMotorInfo(Motors_Base_RF[0], "[Motor_Base_RF]", 21, 3);
    printMotorInfo(Motors_Base_RB[0], "[Motor_Base_RB]", 10, 4);
}

void displayDevices() {
    Brain.Screen.setFont(mono15);
    Brain.Screen.setFillColor(white);

    Brain.Screen.setCursor(4, 1);
    Brain.Screen.setPenColor(Inertial.installed() ? black : red);
    Brain.Screen.print("[Inertial] heading: %.2f port: %d       ", Inertial.heading(deg),
                       18);

    Brain.Screen.setCursor(5, 1);
    Brain.Screen.setPenColor(Rotation_L.installed() ? black : red);
    Brain.Screen.print("[Rotation] deg: %.2f port: %d       ", Rotation_L.position(deg),
                       11);
    Brain.Screen.setCursor(6, 1);
    Brain.Screen.setPenColor(Rotation_R.installed() ? black : red);
    Brain.Screen.print("[Rotation] deg: %.2f port: %d       ", Rotation_R.position(deg),
                       11);
}

void displayInfo() {
    Brain.Screen.setFont(mono20);
    Brain.Screen.setFillColor(white);
    Brain.Screen.setPenColor(black);
    const Point pos = myPosition.getCenterPos();
    Brain.Screen.printAt(0, 3 * 20, "[Position] X: %.2f, Y: %.2f, H: %.2f        ",
                         pos.x_, pos.y_, myPosition.getUnwrappedHeading());
    Brain.Screen.printAt(0, 4 * 20, "[Velocity] Vx: %.2f, Vy: %.2f, Vh: %.2f        ",
                         myPosition.getTranslationalVelocity().dx_,
                         myPosition.getTranslationalVelocity().dy_,
                         myPosition.getAngularVelocity());
}

void displayParams() {
    lbl_translational.render(black, white, white);
    lbl_angular.render(black, white, white);
    lbl_deviational.render(black, white, white);
    lbl_kp.render(black, white, white);
    lbl_ki.render(black, white, white);
    lbl_kd.render(black, white, white);

    btn_param_set_reach_45.render(
        black, black, current_param_set == PIDParamSet::REACH45 ? green : white);
    btn_param_set_reach_90.render(
        black, black, current_param_set == PIDParamSet::REACH90 ? green : white);
    btn_param_set_reach_135.render(
        black, black, current_param_set == PIDParamSet::REACH135 ? green : white);
    btn_param_set_trace.render(black, black,
                               current_param_set == PIDParamSet::TRACE ? green : white);

    const PIDParam* translational_pid = selectedTranslationalPid();
    const PIDParam* angular_pid = selectedAngularPid();

    lbl_translational_kp.setText(translational_pid->kP_);
    lbl_translational_ki.setText(translational_pid->kI_);
    lbl_translational_kd.setText(translational_pid->kD_);
    lbl_angular_kp.setText(angular_pid->kP_);
    lbl_angular_ki.setText(angular_pid->kI_);
    lbl_angular_kd.setText(angular_pid->kD_);

    lbl_translational_kp.render(black, white, white);
    lbl_translational_ki.render(black, white, white);
    lbl_translational_kd.render(black, white, white);
    lbl_angular_kp.render(black, white, white);
    lbl_angular_ki.render(black, white, white);
    lbl_angular_kd.render(black, white, white);

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

    if (current_param_set == PIDParamSet::TRACE) {
        const PIDParam* deviational_pid = selectedDeviationalPid();
        lbl_deviational_kp.setText(deviational_pid->kP_);
        lbl_deviational_ki.setText(deviational_pid->kI_);
        lbl_deviational_kd.setText(deviational_pid->kD_);
        lbl_deviational_kp.render(black, white, white);
        lbl_deviational_ki.render(black, white, white);
        lbl_deviational_kd.render(black, white, white);
        btn_deviational_kp_add.render(black, black, blue);
        btn_deviational_ki_add.render(black, black, blue);
        btn_deviational_kd_add.render(black, black, blue);
        btn_deviational_kp_minus.render(black, black, red);
        btn_deviational_ki_minus.render(black, black, red);
        btn_deviational_kd_minus.render(black, black, red);
    } else {
        lbl_deviational_kp.setText("N/A");
        lbl_deviational_ki.setText("N/A");
        lbl_deviational_kd.setText("N/A");
        lbl_deviational_kp.render(black, white, white);
        lbl_deviational_ki.render(black, white, white);
        lbl_deviational_kd.render(black, white, white);
        btn_deviational_kp_add.render(black, black, color(192, 192, 192));
        btn_deviational_ki_add.render(black, black, color(192, 192, 192));
        btn_deviational_kd_add.render(black, black, color(192, 192, 192));
        btn_deviational_kp_minus.render(black, black, color(192, 192, 192));
        btn_deviational_ki_minus.render(black, black, color(192, 192, 192));
        btn_deviational_kd_minus.render(black, black, color(192, 192, 192));
    }
}

void threadUpdateBrainUI() {
    clearBrain();
    displayTopBar();
    switch (current_interface) {
        case Interface::OPTIONS:
            displayOptions();
            break;
        case Interface::AUTON:
            displayAuton();
            break;
        case Interface::MOTORS:
            displayMotors();
            break;
        case Interface::DEVICES:
            displayDevices();
            break;
        case Interface::INFO:
            displayInfo();
            break;
        case Interface::PARAMS:
            displayParams();
            break;
    }
    while (true) {
        if (current_interface != Interface::OPTIONS) btn_options.check();
        if (current_interface != Interface::AUTON) btn_auton.check();
        if (current_interface != Interface::MOTORS) btn_motors.check();
        if (current_interface != Interface::DEVICES) btn_devices.check();
        if (current_interface != Interface::INFO) btn_info.check();
        if (current_interface != Interface::PARAMS && current_mode == Mode::PID_DEBUG)
            btn_params.check();
        switch (current_interface) {
            case Interface::OPTIONS:
                if (current_driver != Driver::CHIFENG) btn_chifeng.check();
                if (current_driver != Driver::XTR) btn_xtr.check();
                if (current_driver != Driver::CKS) btn_cks.check();
                if (current_driver != Driver::ZYX) btn_zyx.check();

                btn_red.check();
                btn_blue.check();

                if (current_mode != Mode::COMPETITION) btn_competition.check();
                if (current_mode != Mode::PRACTICE) btn_practice.check();
                if (current_mode != Mode::ROUTE_DEBUG) btn_route_debug.check();
                if (current_mode != Mode::PID_DEBUG) btn_pid_debug.check();
                break;
            case Interface::AUTON:
                if (current_route != Route::TOUR1) btn_tour_1.check();
                if (current_route != Route::TOUR2) btn_tour_2.check();
                if (current_route != Route::TOUR3) btn_tour_3.check();
                if (current_route != Route::TOURD) btn_tour_D.check();
                if (current_route != Route::SKILLS_COOP) btn_skills_coop.check();
                if (current_route != Route::SKILLS_SOLO) btn_skills_solo.check();

                if (current_side != Side::LEFT) btn_left.check();
                if (current_side != Side::RIGHT) btn_right.check();
                break;
            case Interface::MOTORS:
                displayMotors();
                break;
            case Interface::DEVICES:
                displayDevices();
                break;
            case Interface::INFO:
                displayInfo();
                break;
            case Interface::PARAMS:
                if (current_param_set != PIDParamSet::REACH45)
                    btn_param_set_reach_45.check();
                if (current_param_set != PIDParamSet::REACH90)
                    btn_param_set_reach_90.check();
                if (current_param_set != PIDParamSet::REACH135)
                    btn_param_set_reach_135.check();
                if (current_param_set != PIDParamSet::TRACE) btn_param_set_trace.check();

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
                if (current_param_set == PIDParamSet::TRACE) {
                    btn_deviational_kp_add.check();
                    btn_deviational_ki_add.check();
                    btn_deviational_kd_add.check();
                    btn_deviational_kp_minus.check();
                    btn_deviational_ki_minus.check();
                    btn_deviational_kd_minus.check();
                }
                break;
        }
        this_thread::sleep_for(cycle.brain_ui_update);
    }
}
