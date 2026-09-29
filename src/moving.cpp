#include "moving.h"
#include "motor.h"
#include <string>
#include <Arduino.h>

/*
TODO:
- Tune PID controller
*/

#define TIME_LIMIT 3000000
#define WAYPOINT_PATIENCE 2         //number of times robot can fail to reach a waypoint
#define STALL_TIMEOUT_MS 2500
#define TURN_PROGRESS_THRESHOLD_DEG 2.0f
#define MOVE_PROGRESS_THRESHOLD_MM 25.0f
#define FORWARD_RECOVERY_DURATION_MS 1000
#define FORWARD_RECOVERY_SPEED 300

IntervalTimer movementTimeLimit;

static moveingState State = IDLE;

//static pos_t lastHandledWeight = {0, 0};

static PID_t rotate(10.5, 0.1, 0.0, 375, 400, 5);
static PID_t translate(10, 0, 0.5, 0, 425, 50);

static pos_t targetlist[MAX_WAYPOINT_COUNT];
static int targetCount = 5;
static int targetIndex = 0;
bool pathfinished = 0;
bool pathBlocked = 0;

static int path_failures = 0;

static pos_t currentTarget = {0,0};
static moveingState monitoredState = IDLE;
static moveingState resumeState = IDLE;
static float bestTargetError = 0.0f;
static uint32_t lastTargetProgressTime = 0;
static uint32_t recoveryEndTime = 0;

static void movementTimerCallback() {
    //if timer finishes before we are at the point reset the state and turn back
    setDriveSpeed(0,0);
    path_failures++;
    State = IDLE;
    movementTimeLimit.end();
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
        if (!isWalkible(x, y, ROBOT_WIDTH-1)) return true;   // now checks the cell just stepped into, not the starting cell
    }
    return false;
}

static float targetError(pos_vector_t pose, moveingState state) {
    if (state == TURNING) {
        float error = findAngle(&pose.pos) - pose.theta / D_TO_R;
        while (error > 180.0f) error -= 360.0f;
        while (error <= -180.0f) error += 360.0f;
        return fabs(error);
    }

    float dx = currentTarget.x - pose.pos.x;
    float dy = currentTarget.y - pose.pos.y;
    return std::sqrt(dx * dx + dy * dy);
}

static void startProgressMonitor(moveingState state, pos_vector_t pose) {
    monitoredState = state;
    bestTargetError = targetError(pose, state);
    lastTargetProgressTime = millis();
}

void findTarget() {
    //get the next target
    if((targetIndex < targetCount-1) && !pathfinished) {
        targetIndex++;
        currentTarget = targetlist[targetIndex];

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
    //BLUETOOTH_SERIAL.println(error);

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
    //BLUETOOTH_SERIAL.println(error);

    //check if we are done
    if (translate.tolerance >= error) return 1;

    int motorSpeed = (int)translate.update(error, dt);
    Serial.println(motorSpeed);
    setDriveSpeed(motorSpeed, motorSpeed);
    return 0;
}

void movementTask(float dt) {

    BLUETOOTH_SERIAL.print("M, "); BLUETOOTH_SERIAL.println(State);

    pos_vector_t myPOS = getMyPOS();

    if (path_failures >= WAYPOINT_PATIENCE) {
        path_failures = 0;
        State = DONE;
    }

    if (State == TURNING || State == MOVEING) {
        if (monitoredState != State) {
            startProgressMonitor(State, myPOS);
        } else {
            float currentError = targetError(myPOS, State);
            float progressThreshold = (State == TURNING)
                ? TURN_PROGRESS_THRESHOLD_DEG
                : MOVE_PROGRESS_THRESHOLD_MM;
            uint32_t now = millis();

            if (currentError + progressThreshold < bestTargetError) {
                bestTargetError = currentError;
                lastTargetProgressTime = now;
            } else if (now - lastTargetProgressTime >= STALL_TIMEOUT_MS) {
                resumeState = State;
                State = RECOVERING;
                recoveryEndTime = now + FORWARD_RECOVERY_DURATION_MS;
                setIsTurning(0);
                movementTimeLimit.end();
                setDriveSpeed(FORWARD_RECOVERY_SPEED, FORWARD_RECOVERY_SPEED);
            }
        }
    }

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
        case RECOVERING:
            if (static_cast<int32_t>(millis() - recoveryEndTime) >= 0) {
                setDriveSpeed(0, 0);
                State = resumeState;
                monitoredState = IDLE;
                if (State == TURNING) {
                    setIsTurning(1);
                } else if (State == MOVEING) {
                    movementTimeLimit.begin(movementTimerCallback, (int)(TIME_LIMIT));
                }
            }
            break;
        case MOVEING:
            if (pathSegmentBlocked(myPOS.pos, currentTarget)) {
                setDriveSpeed(0,0);
                pathBlocked = 1;
                pathfinished = 1;
                State = IDLE;
                movementTimeLimit.end();
                return;
            }
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
    targetIndex = 0;
    State = IDLE;
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
