#ifndef XY_OPTICAL_H
#define XY_OPTICAL_H

#include "Bitcraze_PMW3901.h"

//must be changed if sensor is moved
#define FLOW_THETA_OFFSET -90 //deg
#define FLOW_Y_OFFSET 140 //mm
#define FLOW_X_OFFSET -145

#define XY_CS 10

void init_optical_flow();

void read_opticalFlow(float& deltaX, float& deltaY);

#endif //XY_OPTICAL_H