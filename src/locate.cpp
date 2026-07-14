#include "locate.h"
#include <cmath>
#include <Arduino.h>

#define DEGREES_TO_RADIANS 3.14/180.0f

static pos_t robot_pos = {0,0};

static pos_vector_t current_pos;

static int16_t normalise(float v, int conversion) {
    //normalise a var with a conversion
    return v/conversion;
}

void determineAbsPos(pos_vector_t* pos) {
    //transform from robot coords to world coords



}

pos_vector_t* getCurrentPos() {
    return &current_pos;
}

void updatePos() {
    robot_pos.x = current_pos.pos.x;
    robot_pos.y = current_pos.pos.y;
}

pos_t calcSensorOutputPos(pos_vector_t* sensorMountingOffset, int SensorOutput) {
    //convert sensor coords to robot coords
    pos_t robotCoordTransform;
    float x = sensorMountingOffset->pos.x;
    float y = sensorMountingOffset->pos.y;
    float theta = sensorMountingOffset->theta * DEGREES_TO_RADIANS; //theta of sensors in degrees (easy to measure) cmath sine/cosine wants angle in rad

    robotCoordTransform.x = x + SensorOutput * std::sin(theta);
    robotCoordTransform.y = y + SensorOutput * std::cos(theta);
    return robotCoordTransform;
}

void calcMyLocalPos(int dist, int angle) {
    //convert IMU heading and displacement to robot x, y coords
}