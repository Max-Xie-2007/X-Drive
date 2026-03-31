#include "utils/calc.h"
#include <cmath>

int sgn(const double& val) { return (val > 0) - (val < 0); }

double sat(const double& val, const double& limit) {
    if (std::abs(val) > limit)
        return sgn(val) * limit;
    else
        return val;
}
Vector sat(const Vector& val, const double& limit) {
    if (val.len() > limit)
        return val.norm() * limit;
    else
        return val;
}

double desat(const double& val, const double& limit) {
    if (std::abs(val) < limit)
        return sgn(val) * limit;
    else
        return val;
}
Vector desat(const Vector& val, const double& limit) {
    if (val.len() < limit)
        return val.norm() * limit;
    else
        return val;
}

double deadZone(const double& val, const double& limit) {
    if (std::abs(val) < limit)
        return 0;
    else
        return val;
}
Vector deadZone(const Vector& val, const double& limit) {
    if (val.len() < limit)
        return Vector(0, 0);
    else
        return val;
}
double slew(const double& val, const double& prev_val, const double& slew_rate) {
    double delta = val - prev_val;
    if (std::abs(delta) > slew_rate)
        return prev_val + sgn(delta) * slew_rate;
    else
        return val;
}
Vector slew(const Vector& val, const Vector& prev_val, const double& slew_rate) {
    Vector delta = val - prev_val;
    if (delta.len() > slew_rate)
        return prev_val + delta.norm() * slew_rate;
    else
        return val;
}

double inch2cm(const double& inch) { return inch * 2.54; }
double cm2inch(const double& cm) { return cm / 2.54; }

double deg2rad(const double& angle) { return angle * M_PI / 180.0; }
double rad2deg(const double& angle) { return angle * 180.0 / M_PI; }

double degNorm(const double& angle) {
    double norm_angle = angle;
    while (norm_angle >= 360)
        norm_angle -= 360;
    while (norm_angle < 0)
        norm_angle += 360;
    return norm_angle;
}

double radNorm(const double& angle) {
    double norm_angle = angle;
    while (norm_angle >= 2 * M_PI)
        norm_angle -= 2 * M_PI;
    while (norm_angle < 0)
        norm_angle += 2 * M_PI;
    return norm_angle;
}

double map(const double& x, const double& in_min, const double& in_max,
           const double& out_min, const double& out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double pct2volt(const double& pct) {
    return sgn(pct) * map(std::abs(pct), 0, 100, 0.3, 12.7);
}