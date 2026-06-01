#include "utils/calc.h"

#include <cmath>
#include <stdlib.h>

int sgn(double val) { return (val > 0) - (val < 0); }
double clamp(double val, double min_val, double max_val) {
    if (min_val > max_val) return min_val;
    if (val < min_val)
        return min_val;
    else if (val > max_val)
        return max_val;
    else
        return val;
}
double sat(double val, double limit) {
    if (limit < 0) return 0;
    return clamp(val, -limit, limit);
}
Vector sat(const Vector& val, double limit) {
    if (limit < 0) return Vector(0, 0);
    if (val.len() > limit)
        return val.norm() * limit;
    else
        return val;
}
double deadZone(double val, double limit) {
    if (limit < 0) return 0;
    if (std::abs(val) < limit)
        return 0;
    else
        return val;
}
Vector deadZone(const Vector& val, double limit) {
    if (limit < 0) return Vector(0, 0);
    if (val.len() < limit)
        return Vector(0, 0);
    else
        return val;
}
double slew(double val, double prev_val, double slew_rate) {
    if (slew_rate < 0) return val;
    return clamp(val, prev_val - slew_rate, prev_val + slew_rate);
}
Vector slew(const Vector& val, const Vector& prev_val, double slew_rate) {
    Vector delta = val - prev_val;
    delta = sat(delta, slew_rate);
    return prev_val + delta;
}
double map(double x, double in_min, double in_max, double out_min, double out_max) {
    if (std::fabs(in_max - in_min) < 1e-10) return out_min;
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
double map(double x, double in_min, double in_max, double out_min, double out_max,
           double power) {
    if (std::fabs(in_max - in_min) < 1e-10) return out_min;
    const double base = (x - in_min) / (in_max - in_min);
    return sgn(base) * pow(std::fabs(base), power) * (out_max - out_min) + out_min;
}

double inch2cm(double inch) { return inch * 2.54; }
double cm2inch(double cm) { return cm / 2.54; }

double deg2rad(double angle) { return angle * M_PI / 180.0; }
double rad2deg(double angle) { return angle * 180.0 / M_PI; }

double degNorm360(double angle) {
    while (angle >= 360)
        angle -= 360;
    while (angle < 0)
        angle += 360;
    return angle;
}

double degNorm180(double angle) {
    while (angle >= 180)
        angle -= 360;
    while (angle < -180)
        angle += 360;
    return angle;
}

double radNorm2Pi(double angle) {
    while (angle >= 2 * M_PI)
        angle -= 2 * M_PI;
    while (angle < 0)
        angle += 2 * M_PI;
    return angle;
}

double radNormPi(double angle) {
    while (angle >= M_PI)
        angle -= 2 * M_PI;
    while (angle < -M_PI)
        angle += 2 * M_PI;
    return angle;
}

double pct2volt(double pct) { return sgn(pct) * map(std::abs(pct), 0, 100, 0.3, 12.7); }
