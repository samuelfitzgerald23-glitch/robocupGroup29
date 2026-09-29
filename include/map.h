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

#define BLUETOOTH_SERIAL Serial1

#define MAX_WAYPOINT_COUNT 24
#define WAYPOINT_SPACING 500.0f

#define ROBOT_WIDTH  3          //1/2robot width/hieght in cells
#define MAX_SIGHT_DIST 2000

#define GRID_WIDTH  50
#define GRID_HEIGHT 96

#define CELL_SIZE 50.0f      //mm per cell

//#define X_ZERO (GRID_WIDTH-(2*ROBOT_WIDTH))   //right side
#define X_ZERO (2*ROBOT_WIDTH)              //left side
#define Y_ZERO (2*ROBOT_WIDTH)

#define CONFIDENCE_THRESHOLD -10

#define TOF_CONFIDENCE 1
#define ULTRASOUND_CONFIDENCE 2

enum cell_t {
    EDGE_CELL = -50,
    UNDEFINED_CELL = 0,
    WEIGHT = 50,
    PATH = 51,
    MY_POS = 52
};

struct location_sensor_t {
    BufID ID;
    pos_vector_t offset;            //mounting angle from zero heading (robot frame), x distence & y distence from centre
    PoseMatrix sensor_robotFrame;   //pose matrix used for transforms
    float FOV;
    int8_t confidence;
};

struct GridPos {
    int x, y;
    bool operator==(const GridPos& o) const {return x==o.x && y==o.y;};
};

bool worldToGrid(float world_x, float world_y, int& grid_x, int& grid_y);

void gridToWorld(int gx, int gy, float& world_x, float& world_y);

void writeToMap(float world_x, float world_y, int8_t value);

void updatePose(pos_vector_t* Pose);

void projectSensorCone(const location_sensor_t& sensor, float distance, bool isWeight);

void projectSensorHit(const PoseMatrix& sensor_robotFrame, pos_t* sensorVector, bool isWeight, int8_t confidence);

pos_t projectOpticalFlow(const location_sensor_t& opticalFlow_sensor, pos_t* sensorVector, float theta);

void map_init();

void poseTask();

pos_vector_t getMyPOS();

int8_t getMapValue(int x, int y);

bool isWalkible(int x, int y, int inflation_factor);

bool getweightSeen();

pos_t getLastWeightSeen();

void setweightSeen(bool a);

#endif //MAP_H