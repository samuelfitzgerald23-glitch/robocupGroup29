#include "moving.h"
#include "motor.h"

/*
TODO:
- get my pose
- get target pose
- determine dist between my pose and target
- PID controller
- write to motors

Later:
- auto determine target pose
- convulution map?
- priority of cells
*/

#define MICRO_TO_SEC 1000000

static int lastTime = 0;

static moveingState State = IDLE;

static PID_t rotate(1000, 0, 0, 450, 1);
static PID_t translate(1000, 0, 0, 450, 1);

static pos_t target = {0,0};

static void setTarget(int x, int y) {
    target.x = (float)x;
    target.y = (float)y;
}

void findTarget() {
    //WILL CHANGE LATER
    setTarget(5, 5);
}

float findAngle(pos_t* myPose) {
    //find out error between our current Pose and our target Pose
    float myX = myPose->x;
    float myY = myPose->y;

    float dx = target.x - myX;
    float dy = target.y - myY;

    return std::atan2f(dx,dy)/D_TO_R;
}

bool rotatePID(pos_vector_t* myPose, float dt, PID_t rotate) {
    float error = findAngle(&(myPose->pos)) - myPose->theta;
    //check if we are done
    if (rotate.tolerance >= error) return 1;

    int motorSpeed = (int)rotate.update(error, dt);
    setDriveSpeed(motorSpeed, -motorSpeed);
    return 0;
}

bool translatePID(pos_t* currentXY, float dt, PID_t translate) {
    float dx = target.x - currentXY->x;
    float dy = target.y - currentXY->y;

    float error = std::sqrt(dx*dx + dy*dy);
    //check if we are done
    if (translate.tolerance >= error) return 1;

    int motorSpeed = (int)translate.update(error, dt);
    setDriveSpeed(motorSpeed, motorSpeed);
    return 0;
}

void movementTask() {
    int now = micros();
    float dt = float(now - lastTime);

    pos_vector_t myPOS = getMyPOS();

    switch (State) {
        case IDLE:
            State = TURNING;
            break;
        case TURNING:
            if (rotatePID(&myPOS, dt, rotate)) {setDriveSpeed(0,0); State = MOVEING;}
            break;
        case MOVEING:
            if (translatePID(&(myPOS.pos), dt, translate)) {setDriveSpeed(0,0); State = DONE;}
        case DONE:
            findTarget();
            State = IDLE;
    }
}