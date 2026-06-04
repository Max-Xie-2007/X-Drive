#ifndef UI_BRAIN_H
#define UI_BRAIN_H

#include "ui/ui-base.h"

enum class Interface { OPTIONS, AUTON, MOTORS, DEVICES, INFO, PARAMS };
extern Interface current_interface;

enum class PIDParamSet { REACH45, REACH90, REACH135, TRACE };
extern PIDParamSet current_param_set;

void setInterface(Interface interface);

void clearBrain();

void displayTopBar();

void displayOptions();
void displayAuton();
void displayMotors();
void displayDevices();
void displayInfo();
void displayParams();

void threadUpdateBrainUI();

#endif // UI_BRAIN_H
