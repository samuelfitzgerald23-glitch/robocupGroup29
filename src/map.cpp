#include "map.h"
#include <Arduino.h>

//tof fov in deg
#define TOF_FULL_FOV 27/2               //half of cone angle at max ROI (16x16)
#define TOF_FOV (TOF_FULL_FOV * 1/4)      //half cone angle for 4x4 grid
#define N_TOF_RAYS 5

#define NON_COUPLED_SENSORS 2
#define MAX_WEIGHT_DIST (1/2 * MAX_SIGHT_DIST)
#define WEIGHT_OFFSET 500.0f

static const int8_t dx8[8] = {1, -1, 0, 0, 1, 1, -1, -1};
static const int8_t dy8[8] = {0, 0, 1, -1, 1, -1, 1, -1};

static bool isTurning = 0;

static float prev_heading = 0;

static int8_t mapData[GRID_HEIGHT][GRID_WIDTH];

//filtered sensors with offsets
static location_sensor_t filtered_sensors[NON_COUPLED_SENSORS] = {
    {TOF1, {-5, {105, 170}}, PoseMatrix::buildFrame(&(filtered_sensors[TOF1].offset)), TOF_CONFIDENCE},
    {TOF2, {5, {-105, 170}}, PoseMatrix::buildFrame(&(filtered_sensors[TOF2].offset)), TOF_CONFIDENCE},
    //{ULTRASOUND_LEFT, {}, PoseMatrix::buildFrame(&(filtered_sensors[ULTRASOUND_LEFT].offset)), ULTRASOUND_CONFIDENCE},
    //{ULTRASOUND_RIGHT, {}, PoseMatrix::buildFrame(&(filtered_sensors[ULTRASOUND_RIGHT].offset)), ULTRASOUND_CONFIDENCE},
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
    Serial1.print("C, "); Serial1.print(gx); Serial1.print(", "); Serial1.print(gy);
    Serial1.print(", "); Serial1.println(value);
}

static void logPose(float theta, float x, float y) {
    Serial1.print("P, "); Serial1.print(theta, 4); Serial1.print(", ");
    Serial1.print(x, 2); Serial1.print(", "); Serial1.println(y, 2);
}

bool worldToGrid(float world_x, float world_y, int& grid_x, int& grid_y) {
    //converts world coords to map indeces, then returns true is point is on the map
    int gx = (int)roundf(world_x / CELL_SIZE) + X_ZERO;
    int gy = (int)roundf(world_y / CELL_SIZE) + Y_ZERO;

    bool inBounds = (gx >= 0 && gx < GRID_WIDTH && gy >= 0 && gy < GRID_HEIGHT);

    grid_x = constrain(gx, 0, GRID_WIDTH - 1);
    grid_y = constrain(gy, 0, GRID_HEIGHT - 1);

    return inBounds;
}

void gridToWorld(int gx, int gy, float& world_x, float& world_y) {
    // inverse of worldToGrid
    world_x = (gx - X_ZERO) * CELL_SIZE;
    world_y = (gy - Y_ZERO) * CELL_SIZE;
}

void writeToMap(float world_x, float world_y, int8_t value) {
    //write a value to a cell on the map
    int gx, gy;

    if (worldToGrid(world_x, world_y, gx, gy) && ((mapData[gy][gx] < static_cast<int8_t>(WEIGHT)) && (mapData[gy][gx] > static_cast<int8_t>(EDGE_CELL)))) {
        mapData[gy][gx] = constrain(mapData[gy][gx] + value, static_cast<int8_t>(EDGE_CELL)+1, static_cast<int8_t>(WEIGHT)-1);
        logCell(gx, gy, mapData[gy][gx]);
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
            if ((mapData[y][x] != static_cast<int8_t>(WEIGHT)) && (mapData[y][x] != static_cast<int8_t>(EDGE_CELL))) {
                mapData[y][x] = constrain(mapData[y][x] + 1, static_cast<int8_t>(EDGE_CELL)+1, static_cast<int8_t>(WEIGHT)-1);
                logCell(x, y, mapData[y][x]);
            }
        }
    }
}

/*****************************************************************************************************************************************************************************************
 * projecting pose
 */

void updatePose(pos_vector_t* Pose) {
    //overwrite last pos
    int x, y;
    worldToGrid(robot_worldFrame.x(), robot_worldFrame.y(), x, y);
    mapData[y][x] = 10;

    //get new POS
    robot_worldFrame = robot_worldFrame.transformFrame(PoseMatrix::buildFrame(Pose));

    //write current POS
    int gx, gy;
    worldToGrid(robot_worldFrame.x(), robot_worldFrame.y(), gx, gy);
    mapData[gy][gx] = static_cast<int8_t>(MY_POS);
}

void projectSensorCone(const location_sensor_t& sensor, float distance, bool isWeight) {
    for (int i = 0; i < N_TOF_RAYS; i++) {
        // spread evenly from -halfFOV to +halfFOV
        float angleOffsetDeg = (2.0f * TOF_FOV) * i / (N_TOF_RAYS - 1) - TOF_FOV;
        float angleOffsetRad = angleOffsetDeg * D_TO_R;

        // same polar-to-Cartesian convention as your existing sensorVector (y-forward)
        pos_t rayVector = {
            distance * std::sin(angleOffsetRad),
            distance * std::cos(angleOffsetRad)
        };

        projectSensorHit(sensor.sensor_robotFrame, &rayVector, isWeight, sensor.confidence);
    }
}

void projectSensorHit(const PoseMatrix& sensor_robotFrame, pos_t* sensorVector, bool isWeight, int8_t confidence) {
    //transform sensor frame into world coords
    PoseMatrix sensor_worldFrame = robot_worldFrame.transformFrame(sensor_robotFrame);

    pos_t worldCoord;
    sensor_worldFrame.transformCoords(sensorVector, worldCoord);

    int xr, yr, xw, yw;
    worldToGrid(robot_worldFrame.x(), robot_worldFrame.y(), xr, yr);
    worldToGrid(worldCoord.x, worldCoord.y, xw, yw);
    rayTraceFree(xr, yr, xw, yw);

    if (isWeight) {
        mapData[yw][xw] = static_cast<int8_t>(WEIGHT);
    } else {
        writeToMap(worldCoord.x, worldCoord.y, -confidence);
    }
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

    //write the edge cells
    for (int y = 0; y < GRID_HEIGHT; y++) {
        for (int x = 0; x < GRID_WIDTH; x++) {
            if (x == 0 || x == GRID_WIDTH - 1 || y == 0 || y == GRID_HEIGHT - 1) {
                mapData[y][x] = static_cast<int8_t>(EDGE_CELL);
                logCell(x, y, static_cast<int8_t>(EDGE_CELL));
            }
        }
    }

    //write the starting pos
    writeToMap(robot_worldFrame.x(), robot_worldFrame.y(), static_cast<int8_t>(MY_POS));
}

void poseTask() {
    //Serial.println("pose task");
    float heading = getHeading();
    float differential_heading = prev_heading - heading;
    prev_heading = heading;

    while (differential_heading > 180.0f)  differential_heading -= 360.0f;
    while (differential_heading <= -180.0f) differential_heading += 360.0f;

    //put x and y reading from optical flow sensor into a displacement vector
    pos_t displacement;
    read_opticalFlow(displacement.y, displacement.x);

    pos_vector_t robot_displacement = {
        differential_heading,
        projectOpticalFlow(opticalFlow_sensor, &displacement, differential_heading)
    };

    updatePose(&robot_displacement);

    //find objects with the TOFs
    for (int i = 0; i < NON_COUPLED_SENSORS; i++) {
        if (isTurning || millis() < 1000) continue;

        float sensorDist = (float)getOutput(filtered_sensors[i].ID);
        if (sensorDist > MAX_SIGHT_DIST) continue;

        projectSensorCone(filtered_sensors[i], sensorDist, 0);

        //check to see if seen object is a weight
        switch (filtered_sensors[i].ID) {
            case TOF1:  {
                pos_t out1 = {0 , (float)getOutput(DIFFTOF1)};
                if (out1.y < (sensorDist - WEIGHT_OFFSET)) {
                    //Serial.println("WEIGHT WRITE FIRING");
                    projectSensorHit(filtered_sensors[i].sensor_robotFrame, &out1, 1, 0);
                }
                break;
            }
            case TOF2:  {
                pos_t out2 = {0 , (float)getOutput(DIFFTOF2)};
                if (out2.y < (sensorDist - WEIGHT_OFFSET)) {
                    //Serial.println("WEIGHT WRITE FIRING");
                    projectSensorHit(filtered_sensors[i].sensor_robotFrame, &out2, 1, 0);
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
 * getters/setters
 */

pos_vector_t getMyPOS() {
    return {robot_worldFrame.theta(), {robot_worldFrame.x(), robot_worldFrame.y()}};
}

int8_t getMapValue(int x, int y) {
    return mapData[y][x];
}

void setTurning(bool turning) {
    isTurning = turning;
}

bool isWalkible(int x, int y, int inflation_factor) {
     if (x < 0 || x >= GRID_WIDTH || y < 0 || y >= GRID_HEIGHT) return false;
    if ((getMapValue(x, y) < CONFIDENCE_THRESHOLD) || (getMapValue(x, y) == EDGE_CELL)) return false;

    for (int n = 1; n <= inflation_factor; n++) {
        for (int i = 0; i < 8; i++) {
        int nx = x + n*dx8[i];
        int ny = y + n*dy8[i];
        if (nx < 0 || nx >= GRID_WIDTH || ny < 0 || ny >= GRID_HEIGHT) continue;  // off-grid neighbor isn't a wall, just skip it
        if ((getMapValue(nx, ny) < CONFIDENCE_THRESHOLD) || (getMapValue(nx, ny) == EDGE_CELL)) return false;
    }
    }
    return true;
}