//Author: Samuel Fitzgerald
//Date: 10/7/2026
//Description: functions for transforming world coords to occupancy grid

#ifndef MAP_H
#define MAP_H

#include <cstring>

#include "filtering.h"
#include "frame_positioning.h"
#include "xy_optical.h"
#include "IMU.h"

#define MAX_WAYPOINT_COUNT 24
#define WAYPOINT_SPACING 500.0f

//meters -> decimeters
#define GRID_WIDTH  40
#define GRID_HEIGHT 80

#define CELL_SIZE 100.0f      //mm per cell

#define X_ZERO GRID_WIDTH/2
#define Y_ZERO 1

enum cell_t {
    INVALID_CELL = -3,
    MY_POS,
    VALID_POS,
    UNDEFINED_CELL,
    WEIGHT
};

struct location_sensor_t {
    BufID ID;
    pos_vector_t offset;            //mounting angle from zero heading (robot frame), x distence & y distence from centre
    PoseMatrix sensor_robotFrame;   //pose matrix used for transforms
};

bool worldToGrid(float world_x, float world_y, int& grid_x, int& grid_y);

void gridToWorld(int gx, int gy, float& world_x, float& world_y);

void writeToMap(float world_x, float world_y, int8_t value);

void updatePose(pos_vector_t* Pose);

void projectSensorHit(const PoseMatrix& sensor_robotFrame, pos_t* sensorVector, bool isWeight);

pos_t projectOpticalFlow(const location_sensor_t& opticalFlow_sensor, pos_t* sensorVector, float theta);

void map_init();

void poseTask();

pos_vector_t getMyPOS();

int8_t getMapValue(int x, int y);

#endif //MAP_H