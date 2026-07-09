//Author: Samuel Fitzgerald
//Date: 10/7/2026
//Description: functions for finding stuff

#ifndef LOCATE_H
#define LOCATE_H

#include "filtering.h"

struct pos_t {
    int16_t x;
    int16_t y;
};

struct pos_vector_t {
    int16_t r;
    int16_t theta;
    pos_t abs_pos;
};

void determinePos(pos_vector_t* pos);
pos_vector_t* getCurrentPos();

#endif //LOCATE_H