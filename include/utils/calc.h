#ifndef CALC_H
#define CALC_H
#include "utils/geometry.h"
/**
 * @brief 计算符号函数
 * @param val 输入值
 * @return 正数返回1，负数返回-1，零返回0
 */
int sgn(const double& val);
/**
 * @brief 饱和函数
 * @param val 输入值
 * @param limit 正饱和值
 * @return 饱和后的值
 * @note 若 limit 为负则返回 0
 */
double sat(const double& val, const double& limit);
/**
 * @brief 向量饱和函数
 * @param val 输入向量
 * @param limit 正饱和值
 * @return 饱和后的向量(模长不超过 limit，方向不变)
 * @note 若 limit 为负则返回 0
 */
Vector sat(const Vector& val, const double& limit);
/**
 * @brief 死区函数
 * @param val 输入值
 * @param limit 正死区值
 * @return 0 或死区外的值
 * @note 若 limit 为负则返回 0
 */
double deadZone(const double& val, const double& limit);
/**
 * @brief 向量死区函数
 * @param val 输入向量
 * @param limit 正死区值
 * @return 零向量或模长在死区外的向量
 * @note 若 limit 为负则返回 0
 */
Vector deadZone(const Vector& val, const double& limit);
/**
 * @brief 平滑函数
 * @param val 输入值
 * @param prev_val 前一个输入值
 * @param slew_rate 最大变化率(%/ms)
 * @return 平滑后的值
 */
double slew(const double& val, const double& prev_val, const double& slew_rate);
/**
 * @brief 向量平滑函数
 * @param val 输入向量
 * @param prev_val 前一个输入向量
 * @param slew_rate 最大变化率(%/ms)
 * @return 平滑后的向量
 */
Vector slew(const Vector& val, const Vector& prev_val, const double& slew_rate);
/**
 * @brief 英寸转厘米
 * @param inch 英寸值
 * @return 厘米值
 */
double inch2cm(const double& inch);
/**
 * @brief 厘米转英寸
 * @param cm 厘米值
 * @return 英寸值
 */
double cm2inch(const double& cm);
/**
 * @brief 角度归一化，调整到[0‌°,360‌°‌)
 * @param angle 度数
 * @return 归一化后的角度值
 */
double degNorm360(const double& angle);
/**
 * @brief 角度归一化，调整到(-180‌°,180‌°]
 * @param angle 度数
 * @return 归一化后的角度值
 */
double degNorm180(const double& angle);
/**
 * @brief 弧度归一化，调整到[0,2π)
 * @param angle 弧度值
 * @return 归一化后的弧度值
 */
double radNorm(const double& angle);
/**
 * @brief 度数转弧度
 * @param angle 度数
 * @return 弧度值
 */
double deg2rad(const double& angle);
/**
 * @brief 弧度转度数
 * @param angle 弧度值
 * @return 度数
 */
double rad2deg(const double& angle);

/**
 * @brief 线性映射函数
 * @param x 输入值
 * @param in_min 输入范围最小值
 * @param in_max 输入范围最大值
 * @param out_min 输出范围最小值
 * @param out_max 输出范围最大值
 * @return 映射后的输出值
 */
double map(const double& x, const double& in_min, const double& in_max,
           const double& out_min, const double& out_max);
/**
 * @brief 百分比转电压
 * @param pct 百分比值(-100到100)
 * @return 电压值(VOLT)
 */
double pct2volt(const double& pct);
#endif // CALC_H
