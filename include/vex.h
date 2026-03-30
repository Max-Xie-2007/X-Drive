#ifndef VEX_H
#define VEX_H

#include <cmath>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"

#define waitUntil(condition)                                                             \
    do {                                                                                 \
        wait(5, msec);                                                                   \
    } while (!(condition))
#define waitWhile(condition)                                                             \
    do {                                                                                 \
        wait(5, msec);                                                                   \
    } while (condition);

#define repeat(iterations) for (int iterator = 0; iterator < iterations; iterator++)

#endif // VEX_H