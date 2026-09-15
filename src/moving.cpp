#include "moving.h"
#include "motor.h"
#include <string>
#include <Arduino.h>

/*
TODO:
- Tune PID controller
*/

#define TIME_LIMIT 2000000

IntervalTimer movementTimeLimit;

static moveingState State = IDLE;

static PID_t rotate(25, 1.5, 1, 450, 300, 5);
static PID_t translate(7, 0, 1, 450, 450, 25);

static pos_t targetlist[MAX_WAYPOINT_COUNT] = {{0,0},{0,250},{250,250},{250,0},{0,0}};
static int targetCount = 5;
static int targetIndex = -1;
bool pathfinished = 0;

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

void findTarget() {
    //get the next target
    if((targetIndex < targetCount-1) && !pathfinished) {
        targetIndex++;
        setTarget();
    } else {  
        targetIndex = 0;
        pathfinished = 1;
        // for(int i = 0; i <= 4; i++) {
        //     Serial.println("pos" + String(i) + ":" + String(completedTargetList[i].x) + ", " + String(completedTargetList[i].y));
        // }
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
            if (rotatePID(&myPOS, dt)) {
                setDriveSpeed(0,0); 
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
            delay(1000);
            findTarget();
            State = IDLE;
            break;
    }
}

bool getPathStatus() {
    return pathfinished;
}

void setPath(const pos_t* path, int count) {
    targetCount = count;
    std::copy(path, path + count, targetlist);
    pathfinished = 0;
    for(int i = 0; i < count; i++) {
        float wx = targetlist[i].x; float wy = targetlist[i].y;
        writeToMap(wx, wy, 2);
    }
}