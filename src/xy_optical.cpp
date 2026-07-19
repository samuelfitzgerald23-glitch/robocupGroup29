#include "xy_optical.h"
#include <cmath>

Bitcraze_PMW3901 flow(XY_CS);

void init_optical_flow()
{
    if (!flow.begin()) {
        Serial.println("Initialization of the flow sensor failed");
        while(1) { }
    }
}


//this needs to be changed is the sensor is rotated
void read_opticalFlow(float& deltaX, float& deltaY) {
    //optain raw delta x and y from sensor
    int16_t x, y;
    flow.readMotionCount(&x, &y);

    deltaX = static_cast<float>(x);
    deltaY = static_cast<float>(y);
}