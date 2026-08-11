#include "map.h"
#include <Arduino.h>

static float prev_heading = 0;

static int8_t mapData[GRID_HEIGHT][GRID_WIDTH];

//filtered sensors with offsets
static location_sensor_t filtered_sensors[BUFFERCOUNT] = {
    {TOF1, {-5, {105, 170}}, PoseMatrix::buildFrame(&(filtered_sensors[TOF1].offset))},
    {TOF2, {5, {-105, 170}}, PoseMatrix::buildFrame(&(filtered_sensors[TOF2].offset))}
};

//non filtered sensors with offset
static location_sensor_t opticalFlow_sensor = {NON_FILTERED, {FLOW_THETA_OFFSET, {FLOW_X_OFFSET, FLOW_Y_OFFSET}}, PoseMatrix::buildFrame(&(opticalFlow_sensor.offset))};

//define world origin
static pos_vector_t worldOrigin = {0.0f, {0.0f, 0.0f}};

static PoseMatrix robot_worldFrame = PoseMatrix::buildFrame(&worldOrigin);

/*******************************************************************************************************************************************************************************************
 * converting to grid
 */

static void logCell(int gx, int gy, int8_t value) {
    Serial.print("C, "); Serial.print(gx); Serial.print(", "); Serial.print(gy);
    Serial.print(", "); Serial.println(value);
}

static void logPose(float theta, float x, float y) {
    Serial.print("P, "); Serial.print(theta, 4); Serial.print(", ");
    Serial.print(x, 2); Serial.print(", "); Serial.println(y, 2);
}

bool worldToGrid(float world_x, float world_y, int& grid_x, int& grid_y) {
    //converts world coords to map indeces, then returns true is point is on the map
    grid_x = (int)roundf(world_x / CELL_SIZE) + X_ZERO;
    grid_y = (int)roundf(world_y / CELL_SIZE) + Y_ZERO;

    return(grid_x >= 0 && grid_x < GRID_WIDTH && grid_y >= 0 && grid_y < GRID_HEIGHT);
}

void gridToWorld(int gx, int gy, float& world_x, float& world_y) {
    // inverse of worldToGrid
    world_x = (gx - X_ZERO) * CELL_SIZE;
    world_y = (gy - Y_ZERO) * CELL_SIZE;
}

void writeToMap(float world_x, float world_y, int8_t value) {
    //write a value to a cell on the map
    int gx, gy;

    if (worldToGrid(world_x, world_y, gx, gy) && (mapData[gy][gx] != static_cast<int>(WEIGHT))) {
        mapData[gy][gx] = value;
        logCell(gx, gy, value);
    }
}

static void rayTraceFree(int x0, int y0, int x1, int y1) {
    int dx = abs(x1- x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int error = dx + dy;
    int x = x0, y = y0;

    while (!(x == x1 && y == y1)) {
        int e2 = 2 * error;
        if (e2 >= dy) {error += dy; x += sx;}
        if (e2 <= dx) {error += dx; y += sy;}
        if (x == x1 && y == y1) break;

        if (x >= 0 && x < GRID_WIDTH && y >= 0 && y < GRID_HEIGHT) {
            if (mapData[y][x] != static_cast<int8_t>(WEIGHT)) {
                mapData[y][x] = static_cast<int8_t>(VALID_POS);
                logCell(x, y, static_cast<int8_t>(VALID_POS));
            }
        }
    }
}

/*****************************************************************************************************************************************************************************************
 * projecting pose
 */

void updatePose(pos_vector_t* Pose) {
    //overwrite last pos
    writeToMap(robot_worldFrame.x(), robot_worldFrame.y(), static_cast<int8_t>(VALID_POS));

    //get new POS
    PoseMatrix delta = PoseMatrix::buildFrame(Pose);
    robot_worldFrame = robot_worldFrame.transformFrame(delta);

    //write current POS
    writeToMap(robot_worldFrame.x(), robot_worldFrame.y(), static_cast<int8_t>(MY_POS));
}

void projectSensorHit(const PoseMatrix& sensor_robotFrame, pos_t* sensorVector, bool isWeight) {
    //transform sensor frame into world coords
    PoseMatrix sensor_worldFrame = robot_worldFrame.transformFrame(sensor_robotFrame);

    pos_t worldCoord;
    sensor_worldFrame.transformCoords(sensorVector, worldCoord);

    if (!isWeight) {
        int xr, yr, xw, yw;
        worldToGrid(robot_worldFrame.x(), robot_worldFrame.y(), xr, yr);
        worldToGrid(worldCoord.x, worldCoord.y, xw, yw);
        rayTraceFree(xr, yr, xw, yw);
    }

    writeToMap(worldCoord.x, worldCoord.y, isWeight? static_cast<int8_t>(WEIGHT) : static_cast<int8_t>(INVALID_CELL));
}

pos_t projectOpticalFlow(const location_sensor_t& opticalFlow_sensor, pos_t* sensorVector, float theta) {
    //project the reading from the optical flow sensor into robot coords
    pos_t robotCoord;
    opticalFlow_sensor.sensor_robotFrame.rotateVector(sensorVector, robotCoord);

    //correct for arc rotation caused by sensor offset
    float x = opticalFlow_sensor.offset.pos.x;
    float y = opticalFlow_sensor.offset.pos.y;
    theta = theta * D_TO_R;
    float c = std::cos(theta), s = std::sin(theta);
    float corr_x = (c - 1.0f) * x - s * y;
    float corr_y = s * x + (c - 1.0f) * y;

    robotCoord.x -= corr_x;
    robotCoord.y -= corr_y;

    return robotCoord;
}

/*************************************************************************************************************************************************************************************************
 * overall func
 */

void map_init() {
    std::memset(mapData, static_cast<int8_t>(UNDEFINED_CELL), sizeof(mapData));
}

void poseTask() {
    //get how far robot have moved since the last time we did this
    //heading from IMU, dx and dy from encoders or IR camera or smth
    float heading = getHeading();
    float differential_heading = prev_heading - heading;
    prev_heading = heading;

    //put x and y reading from optical flow sensor into a displacement vector
    pos_t displacement;
    read_opticalFlow(displacement.y, displacement.x);
    //displacement.x *= -1;

    pos_vector_t robot_displacement = {
        differential_heading,
        projectOpticalFlow(opticalFlow_sensor, &displacement, differential_heading)
    };

    updatePose(&robot_displacement);

    //find objects with the TOFs
    for (int i = 0; i < BUFFERCOUNT; i++) {
        pos_t sensorVector = {0, (float)getOutput(filtered_sensors[i].ID)};
        projectSensorHit(filtered_sensors[i].sensor_robotFrame, &sensorVector, 0);

        //check to see if seen object is a weight
        switch (filtered_sensors[i].ID) {
            case TOF1:  {
                int16_t out1 = getOutput(DIFFTOF1);
                if (out1 < (sensorVector.y - 100)) {
                    pos_t diff1 = {0, (float)out1};
                    //diff tofs have the same pose matrix as their parent sensor
                    projectSensorHit(filtered_sensors[i].sensor_robotFrame, &diff1, 1);
                }
                break;
            }
            case TOF2:  {
                int16_t out2 = getOutput(DIFFTOF2);
                if (out2 < (sensorVector.y - 100)) {
                    pos_t diff2 = {0, (float)out2};
                    projectSensorHit(filtered_sensors[i].sensor_robotFrame, &diff2, 1);
                }
                break;
            }
            default:
                break;
        }
    }

    logPose(robot_worldFrame.theta(), robot_worldFrame.x(), robot_worldFrame.y());
}

/*******************************************************************************************************************************************************************************
 * getters
 */

pos_vector_t getMyPOS() {
    return {robot_worldFrame.theta(), {robot_worldFrame.x(), robot_worldFrame.y()}};
}

int8_t getMapValue(int x, int y) {
    return mapData[y][x];
}