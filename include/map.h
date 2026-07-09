//Author: Samuel Fitzgerald
//Date: 10/7/2026
//Description: functions for dealing with and definition of map

#ifndef MAP_H
#define MAP_H

#include <stdint.h>

//meters -> decimeters
#define DELTA 10

#define ARENA_WIDTH 2
#define ARENA_LENGTH 4

#define X_MAX 2 * (ARENA_WIDTH*DELTA) + 1
#define Y_MAX (ARENA_LENGTH*DELTA)

#define X_ZERO ARENA_WIDTH*DELTA

void update_map(int8_t x, int8_t y, int8_t def);
int8_t getPointDef(int8_t x, int8_t y);

#endif //MAP_H