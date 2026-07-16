//Author: Samuel Fitzgerald
//Date: 10/7/2026
//Description: functions for transforming robot coords to occupancy grid

#ifndef MAP_H
#define MAP_H

#include "filtering.h"
#include "frame_positioning.h"

//meters -> decimeters
#define GRID_WIDTH  40
#define GRID_HEIGHT 80

#define CELL_SIZE 0.1f      //metres per cell

#define X_ZERO GRID_WIDTH/2
#define Y_ZERO 1

enum cell_t {
    INVALID_CELL = -1,
    WHERE_IVE_BEEN,
    VALID_POS,
    UNDEFINED_CELL,
    WEIGHT
};

struct location_sensor_t {
    BufID ID;
    pos_vector_t offset;            //mounting angle from zero heading (robot frame), x distence & y distence from centre
    PoseMatrix sensor_robotFrame;   //pose matrix used for transforms
};


#endif //MAP_H