#ifndef CONTROLLER_H
#define CONTROLLER_H

extern int A1, A2, A3, A4, prev_A1, prev_A2, prev_A3, prev_A4;
extern bool L1, L2, R1, R2, X, Y, A, B, Left, Right, Up, Down, prev_L1, prev_L2, prev_R1,
    prev_R2, prev_X, prev_Y, prev_A, prev_B, prev_Left, prev_Right, prev_Up, prev_Down;

void threadUpdateController();

/**
 * @brief 控制器输入曲线参数
 * @param dead_zone 死区范围，输入在此范围内将被视为0
 * @param exponent 曲线指数，控制输入响应的非线性程度
 */
struct ControllerCurveParams {
    double dead_zone = 3.0;
    double exponent = 1.0;
};

double getCurvedOutput(double input, ControllerCurveParams params);

#endif // CONTROLLER_H