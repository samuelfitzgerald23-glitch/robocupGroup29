//Author: Samuel Fitzgerald
//Date: 10/7/2026
//Description: functions for finding stuff

#ifndef LOCATE_H
#define LOCATE_H

#include <stdint.h>
#include <algorithm>

struct pos_t {
    float x;
    float y;
};

struct pos_vector_t {
    float theta;    //can be deg or rad
    pos_t pos;
};

//transform from robot coords to world coords
void determineAbsPos(pos_vector_t* pos);

//transform sensor coords to robot coords
pos_t calcSensorOutputPos(pos_vector_t* sensorMountingOffset, int SensorOutput);

//for total robot only
pos_vector_t* getCurrentPos();
void updateLastPos();


void calcMyLocalPos(int dist, int angle);

#endif //LOCATE_H