#include "xy_optical.h"
#include <cmath>

//must be changed if sensor is moved
#define Y_OFFSET -145 //mm
#define X_OFFSET 140

Bitcraze_PMW3901 flow(XY_CS);

void init_optical_flow()
{
    if (!flow.begin()) {
        Serial.println("Initialization of the flow sensor failed");
        while(1) { }
    }
}


//this needs to be changed is the sensor is rotated
void read_opticalFlow(float& deltaX, float& deltaY, float theta) {
    //optain raw delta x and y from sensor
    int16_t x, y;
    flow.readMotionCount(&x, &y);

    //find correction due to rotation
    float corr_y = (std::cos(theta) - 1) * X_OFFSET - std::sin(theta) * Y_OFFSET;
    float corr_x = std::sin(theta) * X_OFFSET + (std::cos(theta) - 1) * Y_OFFSET;

    //adjust result for displacement caused by rotation
    deltaX = static_cast<float>(x) + corr_x;
    deltaY = static_cast<float>(y) + corr_y;
}