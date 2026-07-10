//Author: Samuel Fitzgerald
//Date: 10/7/2026
//Description: functions for finding stuff

#ifndef LOCATE_H
#define LOCATE_H

#include <stdint.h>

struct pos_t {
    int16_t x;
    int16_t y;
};

struct pos_vector_t {
    int16_t r;
    int16_t theta;
    pos_t abs_pos;
};

//for all pos
void determineAbsPos(pos_vector_t* pos);

//distance sensors only
void calcSensorOutputPos(pos_vector_t* pos, int radialOffset, int angularOffset, int SensorOutput);

//for total robot only
pos_vector_t* getCurrentPos();
void updateLastPos();
void calcMyLocalPos(int dist, int angle);

#endif //LOCATE_H