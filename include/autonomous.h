#ifndef AUTONOMOUS_H
#define AUTONOMOUS_H

/**
 * @brief 左侧 L1 自动程序
 */
void autoL1();
/**
 * @brief 左侧 L2 自动程序
 */
void autoL2();
/**
 * @brief 左侧 L3 自动程序
 */
void autoL3();
/**
 * @brief 右侧 R1 自动程序
 */
void autoR1();
/**
 * @brief 右侧 R2 自动程序
 */
void autoR2();
/**
 * @brief 右侧 R3 自动程序
 */
void autoR3();
/**
 * @brief 技能赛自动程序
 */
void autoSkills();

/**
 * @brief 自动阶段总入口，根据当前路线与颜色分发对应流程
 */
void autonomous();

#endif // AUTO_H