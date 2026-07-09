#include "locate.h"
#include <cmath>

static pos_t last_pos = {0,0};

static pos_vector_t current_pos;

static int16_t normaliseToMap(int16_t v) {
    //normalise mm to decimeters
    return (int16_t)std::round(v/10);
}

void determinePos(pos_vector_t* pos) {
    int16_t r = normaliseToMap(pos->r);
    int16_t theta = pos->theta;
    int16_t x_abs = (int16_t)std::round(std::cos(theta) * r);
    int16_t y_abs = (int16_t)std::round(std::sin(theta) * r);

    pos->abs_pos.x = x_abs + last_pos.x;
    last_pos.x = x_abs;

    pos->abs_pos.y = y_abs + last_pos.y;
    last_pos.y = y_abs;
}

pos_vector_t* getCurrentPos() {
    return &current_pos;
}