#ifndef XY_OPTICAL_H
#define XY_OPTICAL_H

#include "Bitcraze_PMW3901.h"

#define XY_CS 10

void init_optical_flow();

void read_opticalFlow(float& deltaX, float& deltaY, float theta);

#endif //XY_OPTICAL_H