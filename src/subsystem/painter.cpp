#include "subsystem/painter.h"
void startPainting() {
    Piston_Painter.set(true);
}
void stopPainting() {
    Piston_Painter.set(false);
}