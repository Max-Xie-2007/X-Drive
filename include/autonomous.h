#ifndef AUTONOMOUS_H
#define AUTONOMOUS_H
#include "auto-functions/motion.h"
#include "chassis/positioning.h"

#include "ui/ui-brain.h"
#include "ui/ui-controller.h"
#include "utils/counter.h"
void autoL1();
void autoL2();
void autoL3();
void autoLD();
void autoR1();
void autoR2();
void autoR3();
void autoRD();
void autoLSkillsCoop();
void autoRSkillsCoop();
void autoSkillsSolo();

void autonomous();

#endif // AUTO_H