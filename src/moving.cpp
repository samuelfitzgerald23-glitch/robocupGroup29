#include "moving.h"
#include "motor.h"
#include <string>
#include <Arduino.h>

/*
TODO:
- Tune PID controller
*/

#define TIME_LIMIT 2500000

#define SHAKE_SPEED    400        //shake drive speed (-450..450)
#define SHAKE_STEP_MS  120        //ms per move (smaller = sharper shaking)
#define SHAKE_MAX_MM   200.0f     //stop shaking if robot drifts this far from base (0,0)

IntervalTimer movementTimeLimit;

static moveingState State = IDLE;

static PID_t rotate(25, 1.5, 1, 450, 350, 5);
static PID_t translate(8.5, 1, 0, 450, 450, 25);

static pos_t targetlist[MAX_WAYPOINT_COUNT];
static int targetCount = 5;
static int targetIndex = -1;
bool pathfinished = 0;
bool pathBlocked = 0;

static pos_t currentTarget = {0,0};

static void movementTimerCallback() {
    //if timer finishes before we are at the point reset the state and turn back
    setDriveSpeed(0,0);
    State = IDLE;
    movementTimeLimit.end();
}

static void setTarget() {
    //set target to next in path
    currentTarget = targetlist[targetIndex];
}

static bool pathSegmentBlocked(pos_t from, pos_t to) {
    int x0, y0, x1, y1;
    worldToGrid(from.x, from.y, x0, y0);
    worldToGrid(to.x, to.y, x1, y1);

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
    return -1*std::atan2(dx, dy) / D_TO_R;
}

bool rotatePID(pos_vector_t* myPose, float dt) {
    float error = findAngle(&(myPose->pos)) - (myPose->theta)/D_TO_R;
    Serial.print("rotation error: ");

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
    Serial.print("translation error: ");

    //check if we are done
    if (translate.tolerance >= error) return 1;

    int motorSpeed = (int)translate.update(error, dt);
    Serial.println(motorSpeed);
    setDriveSpeed(motorSpeed, motorSpeed);
    return 0;
}

void movementTask(float dt) {
    //if(getRobotState == move) {}

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
            setTurning(1);
            if (rotatePID(&myPOS, dt)) {
                setDriveSpeed(0,0);
                setTurning(0);
                State = MOVEING;
                movementTimeLimit.begin(movementTimerCallback, (int)(TIME_LIMIT));
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

//Multi-direction shake that stays on the base. Reads position each call and
//halts if it strays past SHAKE_MAX_MM, so it can't wander off. Non-blocking -
//call repeatedly from the DROP_PAYLOAD state.
void shakeTask() {
    //bounds check: if we've drifted too far from the base origin, stop
    pos_t p = getMyPOS().pos;
    if (fabsf(p.x) > SHAKE_MAX_MM || fabsf(p.y) > SHAKE_MAX_MM) {
        setDriveSpeed(0, 0);
        return;
    }

    static const int pattern[][2] = {   // each row is {arg1, arg2} for setDriveSpeed()
        {  SHAKE_SPEED,  SHAKE_SPEED },  // lurch forward
        { -SHAKE_SPEED, -SHAKE_SPEED },  // lurch back
        {  SHAKE_SPEED,  SHAKE_SPEED },  // forward again
        { -SHAKE_SPEED, -SHAKE_SPEED },  // back again
        { -SHAKE_SPEED,  SHAKE_SPEED },  // spin one way
        {  SHAKE_SPEED, -SHAKE_SPEED },  // spin the other way
    };
    static const int steps = sizeof(pattern) / sizeof(pattern[0]);

    static uint32_t lastStep = 0;
    static int idx = 0;

    if (millis() - lastStep >= SHAKE_STEP_MS) {
        lastStep = millis();
        setDriveSpeed(pattern[idx][0], pattern[idx][1]);
        idx = (idx + 1) % steps;
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
    for(int i = 0; i < count; i++) {
        float wx = targetlist[i].x; float wy = targetlist[i].y;
        writeToMap(wx, wy, 2);
    }
}
