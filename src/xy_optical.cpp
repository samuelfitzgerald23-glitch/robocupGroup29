#include "xy_optical.h"

//if height of sensor is changed this value will need to be re-calibrated manually
//to calibrate record the number of counts over a known distence (e.g 300mm) and find counts/dist

//at ~12cm off the ground this is 1259 counts / 300 mm
#define COUNTS_PER_MM 4.2f

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

    deltaX = static_cast<float>(x)/COUNTS_PER_MM;
    deltaY = static_cast<float>(y)/COUNTS_PER_MM;
}