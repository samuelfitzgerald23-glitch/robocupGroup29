#include "map.h"
#include <Arduino.h>
#include "reactive.h"

//tof fov in deg
#define TOF_FOV (27.0f/2)      //half cone angle (deg)
#define ULTRASOUND_FOV 15.0f

#define N_RAYS 5

#define NON_COUPLED_SENSORS 4
#define MAX_WEIGHT_DIST (1/2 * MAX_SIGHT_DIST)
#define WEIGHT_OFFSET 200.0f

//FIX: readings below this are sensor errors (0 from a timeout etc), not obstacles
#define MIN_VALID_DIST 30
//FIX: don't map until the filter has run this many times (buffers start full of 0s)
#define MAP_WARMUP_FILTER_UPDATES 5
//FIX: periodic telemetry (T, D, P lines) is sent at most this often
#define TELEMETRY_PERIOD_MS 100
//FIX: map cells are only logged when their value moves into a new band of this
//size (or changes category), instead of on every +/-1
#define LOG_BAND 8


static bool weightSeen = 0;
static pos_t lastWeightSeen;

static bool isTurning = 0;

static float prev_heading = 0;
static bool headingInitialised = false;     //FIX: see poseTask()

static uint32_t lastFilterCount = 0;        //FIX: map each filtered reading once
static uint32_t lastTelemetry = 0;

static int lastPoseGX = -1, lastPoseGY = -1;

static int8_t mapData[GRID_HEIGHT][GRID_WIDTH];

//filtered sensors with offsets
static location_sensor_t filtered_sensors[NON_COUPLED_SENSORS] = {
    {TOF1, {-45, {105, 170}}, PoseMatrix::buildFrame(&(filtered_sensors[0].offset)), TOF_FOV, TOF_CONFIDENCE},
    {TOF2, {45, {-105, 170}}, PoseMatrix::buildFrame(&(filtered_sensors[1].offset)), TOF_FOV, TOF_CONFIDENCE},
    //CHECK: +x is the robot's RIGHT (TOF1 at x=+105 is angled -45 deg = right),
    //so ULTRASOUND_LEFT at x=+65 is on the right. If the LEFT sensor really is on
    //the left, swap these two x offsets (65 <-> -65).
    {ULTRASOUND_LEFT, {0, {65, 170}}, PoseMatrix::buildFrame(&(filtered_sensors[2].offset)), ULTRASOUND_FOV, ULTRASOUND_CONFIDENCE},
    {ULTRASOUND_RIGHT, {0, {-65, 170}}, PoseMatrix::buildFrame(&(filtered_sensors[3].offset)), ULTRASOUND_FOV, ULTRASOUND_CONFIDENCE},
};

//non filtered sensors with offset
static location_sensor_t opticalFlow_sensor = {NON_FILTERED, {FLOW_THETA_OFFSET, {FLOW_X_OFFSET, FLOW_Y_OFFSET}}, PoseMatrix::buildFrame(&(opticalFlow_sensor.offset))};

//define world origin {theta {x, y}}, change theta to match the negetive of starting angle of the robot
//e.g if robot starts turned to right (-90) change theta to 90 to cancel phase shift
static pos_vector_t worldOrigin = {0.0f, {0.0f, 0.0f}};

static PoseMatrix robot_worldFrame = PoseMatrix::buildFrame(&worldOrigin);

/*******************************************************************************************************************************************************************************************
 * converting to grid
 */

static void logCell(int gx, int gy, int8_t value) {
#if MAP_LOGGING
    //FIX: never block. Serial1 (Bluetooth) at 115200 baud only sends ~11 kB/s and
    //write() blocks when its buffer is full. Logging every touched cell every 5 ms
    //was ~5-12 kB per poseTask, so each call took 0.5-1 s and starved the state
    //machine. If there's no room, drop this line (the cell will be re-sent next change).
    if (BLUETOOTH_SERIAL.availableForWrite() < 20) return;
    BLUETOOTH_SERIAL.print("C, "); BLUETOOTH_SERIAL.print(gx); BLUETOOTH_SERIAL.print(", "); BLUETOOTH_SERIAL.print(gy);
    BLUETOOTH_SERIAL.print(", "); BLUETOOTH_SERIAL.println(value);
#endif
}

//only log when a cell changes band or category, so the free-space counters
//climbing 1..49 don't flood the link
static void logCellIfChanged(int gx, int gy, int8_t oldValue, int8_t newValue) {
    if (oldValue == newValue) return;
    bool oldObstacle = oldValue < CONFIDENCE_THRESHOLD;
    bool newObstacle = newValue < CONFIDENCE_THRESHOLD;
    if ((oldValue / LOG_BAND) != (newValue / LOG_BAND) || oldObstacle != newObstacle ||
        oldValue == 0 || newValue == 0) {
        logCell(gx, gy, newValue);
    }
}

static void logPose(float theta, float x, float y) {
    if (BLUETOOTH_SERIAL.availableForWrite() < 32) return;     //FIX: never block
    BLUETOOTH_SERIAL.print("P, "); BLUETOOTH_SERIAL.print(theta, 4); BLUETOOTH_SERIAL.print(", ");
    BLUETOOTH_SERIAL.print(x, 2); BLUETOOTH_SERIAL.print(", "); BLUETOOTH_SERIAL.println(y, 2);
}

bool worldToGrid(float world_x, float world_y, int& grid_x, int& grid_y) {
    //converts world coords to map indeces, then returns true is point is on the map
    int gx = (int)floorf(world_x / CELL_SIZE) + X_ZERO;
    int gy = (int)floorf(world_y / CELL_SIZE) + Y_ZERO;

    bool inBounds = (gx >= 0 && gx < GRID_WIDTH && gy >= 0 && gy < GRID_HEIGHT);

    grid_x = constrain(gx, 0, GRID_WIDTH - 1);
    grid_y = constrain(gy, 0, GRID_HEIGHT - 1);

    return inBounds;
}

void gridToWorld(int gx, int gy, float& world_x, float& world_y) {
    // inverse of worldToGrid
    world_x = (gx - X_ZERO + 0.5f) * CELL_SIZE;
    world_y = (gy - Y_ZERO + 0.5f) * CELL_SIZE;
}

void writeToMap(float world_x, float world_y, int8_t value) {
    //write a value to a cell on the map
    int gx, gy;

    if (worldToGrid(world_x, world_y, gx, gy) && ((mapData[gy][gx] < static_cast<int8_t>(WEIGHT)) && (mapData[gy][gx] > static_cast<int8_t>(EDGE_CELL)))) {
        int8_t old = mapData[gy][gx];
        mapData[gy][gx] = constrain(mapData[gy][gx] + value, static_cast<int8_t>(EDGE_CELL)+1, static_cast<int8_t>(WEIGHT)-1);
        logCellIfChanged(gx, gy, old, mapData[gy][gx]);      //FIX: was logged every time
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
            //FIX: also skip MY_POS / PATH cells (>= WEIGHT), and only log on change
            if ((mapData[y][x] < static_cast<int8_t>(WEIGHT)) && (mapData[y][x] != static_cast<int8_t>(EDGE_CELL))) {
                int8_t old = mapData[y][x];
                mapData[y][x] = constrain(mapData[y][x] + 1, static_cast<int8_t>(EDGE_CELL)+1, static_cast<int8_t>(WEIGHT)-1);
                logCellIfChanged(x, y, old, mapData[y][x]);
            }
        }
    }
}

/*****************************************************************************************************************************************************************************************
 * projecting pose
 */

void updatePose(pos_vector_t* Pose) {
    //get new POS
    robot_worldFrame = robot_worldFrame.transformFrame(PoseMatrix::buildFrame(Pose));

    //FIX: rebuild the matrix from (theta, x, y) so 200 Hz float multiplications
    //can't slowly skew/scale the rotation part
    pos_vector_t clean = {robot_worldFrame.theta() / D_TO_R, {robot_worldFrame.x(), robot_worldFrame.y()}};
    robot_worldFrame = PoseMatrix::buildFrame(&clean);

    //FIX: only touch the map when the robot changes cell, never write outside the
    //map (worldToGrid clamps, which used to overwrite EDGE cells and punch holes in
    //the boundary), and only restore the old cell if it is still our MY_POS marker
    //(it used to overwrite whatever was there - obstacles, weights, edges - with 10)
    int gx, gy;
    if (!worldToGrid(robot_worldFrame.x(), robot_worldFrame.y(), gx, gy)) return;
    if (gx == lastPoseGX && gy == lastPoseGY) return;

    if (lastPoseGX >= 0 && mapData[lastPoseGY][lastPoseGX] == static_cast<int8_t>(MY_POS)) {
        mapData[lastPoseGY][lastPoseGX] = 10;
        logCell(lastPoseGX, lastPoseGY, 10);
    }
    if (mapData[gy][gx] != static_cast<int8_t>(EDGE_CELL)) {
        mapData[gy][gx] = static_cast<int8_t>(MY_POS);
        logCell(gx, gy, static_cast<int8_t>(MY_POS));
    }
    lastPoseGX = gx;
    lastPoseGY = gy;
}

void projectSensorCone(const location_sensor_t& sensor, float distance, bool isWeight) {
    float fov = sensor.FOV;
    for (int i = 0; i < N_RAYS; i++) {
        // spread evenly from -halfFOV to +halfFOV
        float angleOffsetDeg = (2.0f * fov) * i / (N_RAYS - 1) - fov;
        float angleOffsetRad = angleOffsetDeg * D_TO_R;

        // same polar-to-Cartesian convention as your existing sensorVector (y-forward)
        pos_t rayVector = {
            distance * std::sin(angleOffsetRad),
            distance * std::cos(angleOffsetRad)
        };

        //FIX: the sensor reports the NEAREST thing anywhere in its cone, so every ray
        //is free up to that distance, but we don't know WHICH ray hit. Marking all 5
        //endpoints as obstacles put phantom obstacles in free space (worst for the
        //45 deg TOFs looking at walls obliquely). Only the centre ray marks a hit;
        //the side rays just clear free space (confidence 0 = no obstacle write).
        bool centreRay = (i == N_RAYS / 2);
        projectSensorHit(sensor.sensor_robotFrame, &rayVector, isWeight && centreRay,
                         centreRay ? sensor.confidence : 0);
    }
}

void projectSensorHit(const PoseMatrix& sensor_robotFrame, pos_t* sensorVector, bool isWeight, int8_t confidence) {
    //transform sensor frame into world coords
    PoseMatrix sensor_worldFrame = robot_worldFrame.transformFrame(sensor_robotFrame);

    pos_t worldCoord;
    sensor_worldFrame.transformCoords(sensorVector, worldCoord);

    int xr, yr, xw, yw;
    //FIX: worldToGrid clamps off-map points onto the edge. Using the clamped cell
    //traced rays in the wrong direction and could write a WEIGHT over an EDGE cell.
    //Ignore hits that land off the map.
    if (!worldToGrid(robot_worldFrame.x(), robot_worldFrame.y(), xr, yr)) return;
    if (!worldToGrid(worldCoord.x, worldCoord.y, xw, yw)) return;
    rayTraceFree(xr, yr, xw, yw);

    if (isWeight) {
        if (mapData[yw][xw] != static_cast<int8_t>(EDGE_CELL)) {    //FIX: never overwrite the boundary
            if (mapData[yw][xw] != static_cast<int8_t>(WEIGHT)) logCell(xw, yw, static_cast<int8_t>(WEIGHT));
            mapData[yw][xw] = static_cast<int8_t>(WEIGHT);
            weightSeen = 1;
            lastWeightSeen = worldCoord;
        }
    } else if (confidence != 0) {
        writeToMap(worldCoord.x, worldCoord.y, -confidence);
    }
}

pos_t projectOpticalFlow(const location_sensor_t& opticalFlow_sensor, pos_t* sensorVector, float theta) {
    if (isTurning) return {0, 0};

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
    //FIX: wait for a real IMU reading before integrating anything
    if (!IMU_hasData()) return;

    float heading = getHeading();

    //FIX: prev_heading started at 0, so the first update rotated the whole world
    //frame by the IMU's startup heading (e.g. 137 deg) and every goal was off by
    //that much. Take the first reading as the reference instead.
    if (!headingInitialised) {
        prev_heading = heading;
        headingInitialised = true;
    }

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

    bool telemetry = (millis() - lastTelemetry) >= TELEMETRY_PERIOD_MS;
    if (telemetry) lastTelemetry = millis();

    //FIX: only map when the filter has produced NEW readings (every 100 ms).
    //This task runs every 5 ms, so each reading used to be written ~20 times,
    //meaning one bad reading was enough to create an obstacle.
    uint32_t filterCount = getFilterUpdateCount();
    bool newReadings = (filterCount != lastFilterCount);
    lastFilterCount = filterCount;

    //FIX: the old "millis() < 1000" guard never did anything (setup already takes
    //> 1 s). Wait for the filter buffers to fill with real data instead.
    bool warmedUp = filterCount >= MAP_WARMUP_FILTER_UPDATES;

    //FIX: don't map while turning - the sensor data lags the heading by ~100 ms,
    //which smears walls into arcs of phantom obstacles.
    //FIX: the reactive controller doesn't use the map, so don't spend CPU time
    //(and Bluetooth bandwidth) building it. Pose tracking above still runs.
    bool mapping = !USE_REACTIVE_CONTROL;

    if (mapping && newReadings && warmedUp && !isTurning) {
        for (int i = 0; i < NON_COUPLED_SENSORS; i++) {
            float sensorDist = (float)getOutput(filtered_sensors[i].ID);
            if (telemetry && BLUETOOTH_SERIAL.availableForWrite() >= 16) {
                BLUETOOTH_SERIAL.print("T, "); BLUETOOTH_SERIAL.print(i); BLUETOOTH_SERIAL.print(", "); BLUETOOTH_SERIAL.println(sensorDist);
            }

            //FIX: 0 / tiny readings are errors, not an obstacle touching the sensor
            if (sensorDist < MIN_VALID_DIST) continue;

            //check to see if seen object is a weight
            BufID lowerID = NON_FILTERED;
            if (filtered_sensors[i].ID == TOF1) lowerID = DIFFTOF1;
            if (filtered_sensors[i].ID == TOF2) lowerID = DIFFTOF2;

            if (lowerID != NON_FILTERED) {
                pos_t lower = {0, (float)getOutput(lowerID)};
                if (telemetry && BLUETOOTH_SERIAL.availableForWrite() >= 16) {
                    BLUETOOTH_SERIAL.print("D, "); BLUETOOTH_SERIAL.print(i + 2); BLUETOOTH_SERIAL.print(", "); BLUETOOTH_SERIAL.println(lower.y);
                }
                //FIX: lower sensor must have a valid, in-range reading too
                if (lower.y >= MIN_VALID_DIST && lower.y <= MAX_SIGHT_DIST &&
                    lower.y < (sensorDist - WEIGHT_OFFSET)) {
                    projectSensorHit(filtered_sensors[i].sensor_robotFrame, &lower, 1, 0);
                }
            }

            if (sensorDist > MAX_SIGHT_DIST) continue;
            projectSensorCone(filtered_sensors[i], sensorDist, 0);
        }
    }

    if (telemetry) logPose(robot_worldFrame.theta(), robot_worldFrame.x(), robot_worldFrame.y());
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

bool isWalkible(int x, int y, int inflation_factor) {
    if (x < 0 || x >= GRID_WIDTH || y < 0 || y >= GRID_HEIGHT) return false;
    if ((getMapValue(x, y) < CONFIDENCE_THRESHOLD)) return false;

    //FIX: check the whole square around the cell. The old version only checked
    //the 8 compass directions (a star), so an obstacle at e.g. (+1,+2) was never
    //seen, even at inflation 7 - paths clipped obstacles.
    for (int dy = -inflation_factor; dy <= inflation_factor; dy++) {
        for (int dx = -inflation_factor; dx <= inflation_factor; dx++) {
            int nx = x + dx;
            int ny = y + dy;
            if (nx < 0 || nx >= GRID_WIDTH || ny < 0 || ny >= GRID_HEIGHT) return false;
            if ((getMapValue(nx, ny) < CONFIDENCE_THRESHOLD)) return false;
        }
    }
    return true;
}

bool getweightSeen() {
    return weightSeen;
}

pos_t getLastWeightSeen() {
    return lastWeightSeen;
}

void setweightSeen(bool a) {
    weightSeen = a;
}

void setIsTurning(bool a) {
    isTurning = a;
}