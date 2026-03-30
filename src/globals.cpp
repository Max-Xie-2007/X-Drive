#include "globals.h"

#ifdef LEFT
Route current_route = L1;
std::string auto_route_name[4] = {"L1", "L2", "L3", "SKILLS"};
bool expansion_complete = true;
#endif
#ifdef RIGHT
Route current_route = R1;
std::string auto_route_name[4] = {"R1", "R2", "R3", "SKILLS"};
bool expansion_complete = false;
#endif

Color current_color = RED;

// Mode current_mode = COMPETITION;
Mode current_mode = PID_DEBUG;
// Mode current_mode = ROUTE_DEBUG;

void changeModeTo(Mode new_mode) { current_mode = new_mode; }
void changeRouteTo(Route new_route) { current_route = new_route; }
void changeColor() { current_color = (current_color == RED) ? BLUE : RED; }

void switchRoute() {
    int next_val = (static_cast<int>(current_route) + 1) % 4;
    current_route = static_cast<Route>(next_val);
}