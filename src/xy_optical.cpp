#include "xy_optical.h"


Bitcraze_PMW3901 flow(XY_CS);

void init_optical_flow()
{
    if (!flow.begin()) {
        Serial.println("Initialization of the flow sensor failed");
        while(1) { }
    }
}

void read_opticalFlow(float& deltaX, float& deltaY) {
    int16_t x, y;
    flow.readMotionCount(&x, &y);
    deltaX = static_cast<float>(x);
    deltaY = static_cast<float>(y);
}