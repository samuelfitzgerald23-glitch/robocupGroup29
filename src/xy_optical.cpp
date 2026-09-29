#include "xy_optical.h"

//if height of sensor is changed this value will need to be re-calibrated manually
//to calibrate record the number of counts over a known distence (e.g 300mm) and find counts/dist

//at ~12cm off the ground this is 1259 counts / 300 mm
//CHECK: 1259/300 = 4.20, but the value below is 2.82 (= 846 counts / 300 mm).
//If 4.20 is right, every distance is over-estimated by ~49% and the robot stops
//short of every waypoint. Re-measure: push the robot 1000 mm along a tape measure
//and divide the total counts by 1000.
#define COUNTS_PER_MM 2.82f

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
    //FastLED.show();
}