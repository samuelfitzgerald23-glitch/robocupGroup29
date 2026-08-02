#include "moving.h"
#include "motor.h"
#include <string>

/*
TODO:
- Tune PID controller
*/

#define MICRO_TO_SEC 1000000
#define TIME_LIMIT 2*MICRO_TO_SEC

IntervalTimer movementTimeLimit;

static int lastTime = 0;

static moveingState State = IDLE;

static PID_t rotate(100, 0, 0, 350, 5);
static PID_t translate(100, 0, 0, 350, 50);

static pos_t targetlist[5] = {{0,0}, {0, 200}, {200, 200}, {200, 0}, {0, 0}};                                
static int targetIndex = -1;
bool pathfinished = 0;

static pos_t currentTarget = {0,0};

static pos_t completedTargetList[5];

static void movementTimerCallback() {
    //if timer finishes before we are at the point reset the state and turn back
    setDriveSpeed(0,0);
    State = TURNING;
    movementTimeLimit.end();
}

static void setTarget() {
    //set target to next in path
    currentTarget = targetlist[targetIndex];
}

void findTarget() {
    //get the next target
    if((targetIndex < 4) && !pathfinished) {
        targetIndex++;
        setTarget();
    } else {  
        targetIndex = 0;
        pathfinished = 1;
        for(int i = 0; i <= 4; i++) {
            Serial.println("pos" + String(i) + ":" + String(completedTargetList[i].x) + ", " + String(completedTargetList[i].y));
        }
    }
}

float findAngle(pos_t* myPose) {
    float dx = currentTarget.x - myPose->x;
    float dy = currentTarget.y - myPose->y;
    return -1*std::atan2f(dx, dy) / D_TO_R;
}

bool rotatePID(pos_vector_t* myPose, float dt, PID_t& rotate) {
    float error = findAngle(&(myPose->pos)) - (myPose->theta)/D_TO_R;

    while (error > 180.0f)  error -= 360.0f;
    while (error <= -180.0f) error += 360.0f;

    //check if we are done
    if (rotate.tolerance >= fabs(error)) return 1;

    int motorSpeed = (int)rotate.update(error, dt);
    setDriveSpeed(-motorSpeed, motorSpeed);
    return 0;
}

bool translatePID(pos_t* currentXY, float dt, PID_t& translate) {
    float dx = currentTarget.x - currentXY->x;
    float dy = currentTarget.y - currentXY->y;
    float error = std::sqrt(dx*dx + dy*dy);

    //check if we are done
    if (translate.tolerance >= error) return 1;

    int motorSpeed = (int)translate.update(error, dt);
    setDriveSpeed(motorSpeed, motorSpeed);
    return 0;
}

void movementTask() {
    int now = micros();
    float dt = float(now - lastTime)/MICRO_TO_SEC;
    lastTime = now;

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
            if (rotatePID(&myPOS, dt, rotate)) {
                setDriveSpeed(0,0); 
                State = MOVEING;
                movementTimeLimit.begin(movementTimerCallback, (int)(TIME_LIMIT));
            }
            break;
        case MOVEING:
            if (translatePID(&(myPOS.pos), dt, translate)) {
                setDriveSpeed(0,0); 
                State = DONE;
                movementTimeLimit.end();
            }
            break;
        case DONE:
            findTarget();
            completedTargetList[targetIndex] = myPOS.pos;
            State = IDLE;
            delay(10000);
            break;
    }
}

bool getPathStatus() {
    return pathfinished;
}