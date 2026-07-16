#include "map.h"
#include <Arduino.h>

static int8_t mapData[GRID_HEIGHT][GRID_WIDTH];

static location_sensor_t filtered_sensors[BUFFERCOUNT] = {
    {TOF1, {0, {-10, 10}}, PoseMatrix::buildFrame(&(filtered_sensors[0].offset))},
    {TOF2, {0, {10, 10}}, PoseMatrix::buildFrame(&(filtered_sensors[0].offset))}
};

static pos_vector_t worldOrigin = {0.0f, {0.0f, 0.0f}};

static PoseMatrix robot_worldFrame;

bool worldToGrid(float world_x, float world_y, int& grid_x, int& grid_y) {
    //converts world coords to map indeces, then returns true is point is on the map
    grid_x = (int)roundf(world_x / CELL_SIZE) + X_ZERO;
    grid_y = (int)roundf(world_y / CELL_SIZE) + Y_ZERO;
    return(grid_x >= 0 && grid_x < GRID_WIDTH && grid_y >= 0 && grid_y < GRID_HEIGHT);
}

void writeToMap(float world_x, float world_y, int8_t value) {
    int gx, gy;
    if (worldToGrid(world_x, world_y, gx, gy)) mapData[gy][gx] = value;
}

void updatePose(pos_vector_t* Pose) {
    //get new POS
    PoseMatrix delta = PoseMatrix::buildFrame(Pose);
    robot_worldFrame = robot_worldFrame.transformFrame(delta);

    //write current POS
    writeToMap(robot_worldFrame.x(), robot_worldFrame.y(), static_cast<int8_t>(WHERE_IVE_BEEN));
}

void projectSensorHit(const PoseMatrix& sensor_robotFrame, pos_t* sensorVector, bool isWeight) {
    //transform sensor frame into world coords
    PoseMatrix sensor_worldFrame = robot_worldFrame.transformFrame(sensor_robotFrame);

    pos_t worldCoord;
    sensor_worldFrame.transformCoords(sensorVector, worldCoord);

    writeToMap(worldCoord.x, worldCoord.y, isWeight? static_cast<int8_t>(WEIGHT) : static_cast<int8_t>(INVALID_CELL));
}

void navigateTask() {
    pos_vector_t robot_displacement = {0, {0, 0.1}};
    updatePose(&robot_displacement);

    for (int i = 0; i < BUFFERCOUNT; i++) {
        pos_t sensorVector = {0, getOutput(filtered_sensors[i].ID)};

        projectSensorHit(filtered_sensors[i].sensor_robotFrame, &sensorVector, 0);
    }
}