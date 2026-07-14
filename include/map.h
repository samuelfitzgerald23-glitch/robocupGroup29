//Author: Samuel Fitzgerald
//Date: 10/7/2026
//Description: functions for dealing with and definition of map

#ifndef MAP_H
#define MAP_H

#include "filtering.h"
#include "locate.h"

//meters -> decimeters
#define DELTA 1

#define ARENA_WIDTH 2
#define ARENA_LENGTH 4

#define X_MAX 2 * (ARENA_WIDTH*DELTA) + 1
#define Y_MAX (ARENA_LENGTH*DELTA) + 1

#define X_ZERO ARENA_WIDTH*DELTA
#define Y_ZERO 1

enum cell_t {
    INVALID_CELL = -1,
    MY_POS,
    VALID_POS,
    UNDEFINED_CELL,
    WEIGHT
};

struct location_sensor_t {
    int16_t SensorOutput;
    BufID ID;
    int radialOffset;   //mm
    int angularOffset;  //deg
};

void update_map(pos_t* pos, int8_t def);
int16_t getPointDef(pos_t* pos);

void findWalls();

void locationTask();

#endif //MAP_H