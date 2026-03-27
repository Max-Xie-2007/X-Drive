#include "vex.h"

#ifdef LEFT
Route current_route = L1;
std::string auto_route_name[4] = {"L1", "L2", "L3", "SKILLS"};
#endif
#ifdef RIGHT
Route current_route = R1;
std::string auto_route_name[4] = {"R1", "R2", "R3", "SKILLS"};
#endif

Color current_color = RED;

// Mode current_mode = COMPETITION;
Mode current_mode = PID_DEBUG;
// Mode current_mode = ROUTE_DEBUG;