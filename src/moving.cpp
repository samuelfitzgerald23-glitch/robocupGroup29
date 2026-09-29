#include "moving.h"
#include "motor.h"
#include <string>
#include <Arduino.h>

/*
TODO:
- Tune PID controller
*/

#define TIME_LIMIT 3000000
#define WAYPOINT_PATIENCE 3         //number of times robot can fail to reach a waypoint

IntervalTimer movementTimeLimit;

static moveingState State = IDLE;

//static pos_t lastHandledWeight = {0, 0};

static PID_t rotate(25, 0.5, 0, 450, 350, 5);
static PID_t translate(7, 1, 0, 450, 425, 50);

static pos_t targetlist[MAX_WAYPOINT_COUNT];
static int targetCount = 5;
static int targetIndex = 0;
bool pathfinished = 0;
bool pathBlocked = 0;

static int path_failures = 0;

static pos_t currentTarget = {0,0};

static void movementTimerCallback() {
    //if timer finishes before we are at the point reset the state and turn back
    setDriveSpeed(0,0);
    path_failures++;
    State = IDLE;
    movementTimeLimit.end();
}

static void setTarget() {
    //set target to next in path
    currentTarget = targetlist[targetIndex];
}

static bool pathSegmentBlocked(pos_t from, pos_t to) {
    int x0, y0, x1, y1;
    if(!worldToGrid(from.x, from.y, x0, y0) || !worldToGrid(to.x, to.y, x1, y1)) return true;

    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int error = dx + dy;
    int x = x0, y = y0;

    while (!(x == x1 && y == y1)) {
        int e2 = 2 * error;
        if (e2 >= dy) { error += dy; x += sx; }
        if (e2 <= dx) { error += dx; y += sy; }
        if (!isWalkible(x, y, ROBOT_WIDTH)) return true;   // now checks the cell just stepped into, not the starting cell
    }
    return false;
}

void findTarget() {
    //get the next target
    if((targetIndex < targetCount-1) && !pathfinished) {
        targetIndex++;
        setTarget();

    } else {  
        pathfinished = 1;
        targetIndex = 0;
    }
}

float findAngle(pos_t* myPose) {
    float dx = currentTarget.x - myPose->x;
    float dy = currentTarget.y - myPose->y;
    return -std::atan2(dx, dy) / D_TO_R;
}

bool rotatePID(pos_vector_t* myPose, float dt) {
    float error = findAngle(&(myPose->pos)) - (myPose->theta)/D_TO_R;
    BLUETOOTH_SERIAL.println(error);

    while (error > 180.0f)  error -= 360.0f;
    while (error <= -180.0f) error += 360.0f;

    //check if we are done
    if (rotate.tolerance >= fabs(error)) return 1;

    int motorSpeed = (int)rotate.update(error, dt);
    Serial.println(motorSpeed);
    setDriveSpeed(-motorSpeed, motorSpeed);
    return 0;
}

bool translatePID(pos_t* currentXY, float dt) {
    float dx = currentTarget.x - currentXY->x;
    float dy = currentTarget.y - currentXY->y;
    float error = std::sqrt(dx*dx + dy*dy);
    BLUETOOTH_SERIAL.println(error);

    //check if we are done
    if (translate.tolerance >= error) return 1;

    int motorSpeed = (int)translate.update(error, dt);
    Serial.println(motorSpeed);
    setDriveSpeed(motorSpeed, motorSpeed);
    return 0;
}

void movementTask(float dt) {

    pos_vector_t myPOS = getMyPOS();

    if (pathSegmentBlocked(myPOS.pos, currentTarget)) {
        pathfinished = 1;   // signals State.cpp to go back through PATHFIND
        pathBlocked = 1;
        targetIndex = 0;
        setDriveSpeed(0,0);
        State = IDLE;
        movementTimeLimit.end();
        return;
    }
    if (path_failures >= WAYPOINT_PATIENCE) {
        path_failures = 0;
        State = DONE;
    }

    // if (getweightSeen()) {
    //     pos_t seenWeight = getLastWeightSeen();
    //     bool isNewWeight = (seenWeight.x != lastHandledWeight.x) || (seenWeight.y != lastHandledWeight.y);

    //     if (isNewWeight) {
    //         movementTimeLimit.end();
    //         setDriveSpeed(0, 0);
    //         currentTarget = seenWeight;
    //         lastHandledWeight = seenWeight;
    //         State = IDLE;
    //     }
    //     setweightSeen(0);
    // }
    BLUETOOTH_SERIAL.print("e, ");
    switch (State) {
        case IDLE:
            {
            rotate.reset();
            translate.reset();

            //if we are too close to target choose a new one
            float dx = currentTarget.x - myPOS.pos.x;
            float dy = currentTarget.y - myPOS.pos.y;
            float dist = std::sqrt(dx*dx + dy*dy);

            State = (dist <= translate.tolerance) ? DONE : TURNING;
            break;
        }
        case TURNING:
            setIsTurning(1);
            if (rotatePID(&myPOS, dt)) {
                setDriveSpeed(0,0);
                State = MOVEING;
                movementTimeLimit.begin(movementTimerCallback, (int)(TIME_LIMIT));
                setIsTurning(0);
            }
            break;
        case MOVEING:
            if (translatePID(&(myPOS.pos), dt)) {
                setDriveSpeed(0,0); 
                State = DONE;
                movementTimeLimit.end();
            }
            break;
        case DONE:
            findTarget();
            State = IDLE;
            break;
    }
}

bool getPathStatus() {
    return pathfinished;
}

bool getPathBlockedStatus() {
    return pathBlocked;
}

void setPath(const pos_t* path, int count) {
    targetCount = count;
    std::copy(path, path + count, targetlist);
    pathfinished = 0;
    pathBlocked = 0;
    currentTarget = targetlist[0];
    for (int i = 0; i < count; i++) {
        int gx, gy;
        worldToGrid(targetlist[i].x, targetlist[i].y, gx, gy);
        BLUETOOTH_SERIAL.print("C, "); BLUETOOTH_SERIAL.print(gx); BLUETOOTH_SERIAL.print(", "); BLUETOOTH_SERIAL.print(gy);
        BLUETOOTH_SERIAL.print(", "); BLUETOOTH_SERIAL.println(52);
    }
}
