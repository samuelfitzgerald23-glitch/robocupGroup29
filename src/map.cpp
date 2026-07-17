#include "map.h"
#include <Arduino.h>

static int8_t mapData[GRID_HEIGHT][GRID_WIDTH];

static location_sensor_t filtered_sensors[BUFFERCOUNT] = {
    {TOF1, {0, {-10, 10}}, PoseMatrix::buildFrame(&(filtered_sensors[0].offset))},
    {TOF2, {0, {10, 10}}, PoseMatrix::buildFrame(&(filtered_sensors[0].offset))}
};

static pos_vector_t worldOrigin = {0.0f, {0.0f, 0.0f}};

static PoseMatrix robot_worldFrame = PoseMatrix::buildFrame(&worldOrigin);

bool worldToGrid(float world_x, float world_y, int& grid_x, int& grid_y) {
    //converts world coords to map indeces, then returns true is point is on the map
    grid_x = (int)roundf(world_x / CELL_SIZE) + X_ZERO;
    grid_y = (int)roundf(world_y / CELL_SIZE) + Y_ZERO;

    Serial.print(" x= ");
    Serial.print(grid_x);

    Serial.print(" y= ");
    Serial.println(grid_y);

    return(grid_x >= 0 && grid_x < GRID_WIDTH && grid_y >= 0 && grid_y < GRID_HEIGHT);
}

void writeToMap(float world_x, float world_y, int8_t value) {
    int gx, gy;

    Serial.print("point defined as: ");
    Serial.print(value);
    Serial.print(" at:");

    if (worldToGrid(world_x, world_y, gx, gy)) mapData[gy][gx] = value;
}

void updatePose(pos_vector_t* Pose) {
    //overwrite last pos
    writeToMap(robot_worldFrame.x(), robot_worldFrame.y(), static_cast<int8_t>(VALID_POS));

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
    //get how far robot have moved since the last time we did this
    //heading from IMU, dx and dy from encoders or IR camera or smth
    pos_vector_t robot_displacement = {0, {0, 0}};                          //initialise the robots pos with respect to its last pos
    read_opticalFlow(robot_displacement.pos.x, robot_displacement.pos.y);   //get x and y from optical encoder

    updatePose(&robot_displacement);

    //find objects with the TOFs
    for (int i = 0; i < BUFFERCOUNT; i++) {
        pos_t sensorVector = {0, (float)getOutput(filtered_sensors[i].ID)};

        projectSensorHit(filtered_sensors[i].sensor_robotFrame, &sensorVector, 0);
    }
}