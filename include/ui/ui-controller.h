#ifndef UI_CONTROLLER_H
#define UI_CONTROLLER_H

void clearController();

void printPreFoldPage();
void printConfirmationPage();
void printCalibrationPage();
void printSensorValues();
void printPosition();
void printRouteNColor();

void threadUpdateControllerUI();

#endif // UI_CONTROLLER_H
