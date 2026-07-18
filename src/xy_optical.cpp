#include "xy_optical.h"
#include <cmath>

#define Y_OFFSET 10 //mm
#define X_OFFSET 10

Bitcraze_PMW3901 flow(XY_CS);

void init_optical_flow()
{
    if (!flow.begin()) {
        Serial.println("Initialization of the flow sensor failed");
        while(1) { }
    }
}

void read_opticalFlow(float& deltaX, float& deltaY, float theta) {
    //optain raw delta x and y from sensor
    int16_t x, y;
    flow.readMotionCount(&x, &y);

    //find correction due to rotation
    corr_x = (std::cos(theta) - 1) * X_OFFSET - std::sin(theta) * Y_OFFSET;
    corr_y = std::sin(theta) * X_OFFSET + (std::cos(theta) - 1) * Y_OFFSET;

    //adjust result for displacement caused by rotation
    deltaX = static_cast<float>(x) - corr_x;
    deltaY = static_cast<float>(y) - corr_y;
}