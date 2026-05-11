#ifndef VEX_H
#define VEX_H

#include "v5.h"
#include "v5_vcs.h"

#include <cmath>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define waitUntil(condition)                                                             \
    do {                                                                                 \
        wait(5, msec);                                                                   \
    } while (!(condition))
#define waitWhile(condition)                                                             \
    do {                                                                                 \
        wait(5, msec);                                                                   \
    } while (condition);

#define repeat(iterations) for (int iterator = 0; iterator < iterations; iterator++)

enum DriveMethod { VOLT, PCT };

#endif // VEX_H
