#include "map.h"
#include <Arduino.h>

static float prev_heading = 0;

static int8_t mapData[GRID_HEIGHT][GRID_WIDTH];

//filtered sensors with offsets
static location_sensor_t filtered_sensors[BUFFERCOUNT] = {
    {TOF1, {0, {-100, 136}}, PoseMatrix::buildFrame(&(filtered_sensors[0].offset))},
    {TOF2, {0, {100, 136}}, PoseMatrix::buildFrame(&(filtered_sensors[0].offset))}
};

//non filtered sensors with offset
static location_sensor_t opticalFlow_sensor = {NON_FILTERED, {FLOW_THETA_OFFSET, {FLOW_X_OFFSET, FLOW_Y_OFFSET}}, PoseMatrix::buildFrame(&(opticalFlow_sensor.offset))};

//define world origin
static pos_vector_t worldOrigin = {0.0f, {0.0f, 0.0f}};

static PoseMatrix robot_worldFrame = PoseMatrix::buildFrame(&worldOrigin);

bool worldToGrid(float world_x, float world_y, int& grid_x, int& grid_y) {
    //converts world coords to map indeces, then returns true is point is on the map
    grid_x = (int)roundf(world_x / CELL_SIZE) + X_ZERO;
    grid_y = (int)roundf(world_y / CELL_SIZE) + Y_ZERO;

    Serial.print(" x = ");
    Serial.print(grid_x);

    Serial.print(" y = ");
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

pos_t projectOpticalFlow(const PoseMatrix& optical_robotFrame, pos_t* sensorVector) {
    //project the reading from the optical flow sensor into robot coords
    pos_t robotCoord;
    optical_robotFrame.rotateVector(sensorVector, robotCoord);
    return robotCoord;
}

void poseTask() {
    //get how far robot have moved since the last time we did this
    //heading from IMU, dx and dy from encoders or IR camera or smth
    float heading = getHeading();
    float differential_heading = prev_heading - heading;
    prev_heading = heading;

    //put x and y reading from optical flow sensor into a displacement vector
    pos_t displacement;
    read_opticalFlow(displacement.x, displacement.y);
    Serial.print(displacement.x);
    Serial.print(displacement.y);

    pos_vector_t robot_displacement = {differential_heading, projectOpticalFlow(opticalFlow_sensor.sensor_robotFrame, &displacement)};

    Serial.println("Robot coords");
    Serial.print("heading: ");
    Serial.print(differential_heading);
    Serial.print(", x: ");
    Serial.print(robot_displacement.pos.x);
    Serial.print(", y: ");
    Serial.println(robot_displacement.pos.y);

    updatePose(&robot_displacement);

    //find objects with the TOFs
    for (int i = 0; i < BUFFERCOUNT; i++) {
        pos_t sensorVector = {0, (float)getOutput(filtered_sensors[i].ID)};

        projectSensorHit(filtered_sensors[i].sensor_robotFrame, &sensorVector, 0);
    }

    Serial.println("World coords");
    Serial.print("heading: ");
    Serial.print(robot_worldFrame.theta());
    Serial.print(", x: ");
    Serial.print(robot_worldFrame.x());
    Serial.print(", y: ");
    Serial.println(robot_worldFrame.y());
}