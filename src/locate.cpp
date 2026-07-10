#include "locate.h"
#include <cmath>

static pos_t last_pos = {0,0};

static pos_vector_t current_pos;

static int16_t normaliseToMap(int16_t v, int conversion) {
    //normalise mm to decimeters
    return (int16_t)std::round(v/conversion);
}

void determineAbsPos(pos_vector_t* pos) {
    int16_t r = normaliseToMap(pos->r, 10);
    int16_t theta = pos->theta;
    int16_t x_abs = (int16_t)std::round(std::cos(theta) * r) + last_pos.x;
    int16_t y_abs = (int16_t)std::round(std::sin(theta) * r) + last_pos.y;

    pos->abs_pos.x = x_abs;

    pos->abs_pos.y = y_abs;

    if (pos == &current_pos) updateLastPos();
}

pos_vector_t* getCurrentPos() {
    return &current_pos;
}

void updateLastPos() {
    last_pos.x = current_pos.abs_pos.x;
    last_pos.y = current_pos.abs_pos.y;
}

void calcSensorOutputPos(pos_vector_t* pos, int radialOffset, int angularOffset, int SensorOutput) {
    //find the local pos in radial coords of a sensor output
    int16_t r = std::round(std::sqrt((radialOffset * radialOffset) + (SensorOutput * SensorOutput) + 2 * radialOffset * SensorOutput * std::cos(180-angularOffset)));
    int16_t theta = std::round(std::acos(((r*r) + (SensorOutput*SensorOutput) - (radialOffset*radialOffset))/(2*radialOffset*SensorOutput)));

    pos->r = r;
    pos->theta = theta;
}

void calcMyLocalPos(int dist, int angle) {
    current_pos.r = dist;
    current_pos.theta = angle;
}