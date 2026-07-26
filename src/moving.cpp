#include "moving.h"
#include "motor.h"

/*
TODO:
- determine dist between my pose and target
- PID controller

Later:
- auto determine target pose
- convulution map?
- priority of cells
*/

#define MICRO_TO_SEC 1000000

static int lastTime = 0;

static moveingState State = IDLE;

static PID_t rotate(10, 0, 0, 350, 5);
static PID_t translate(10, 0, 0, 350, 2);

static pos_t target = {0,0};

static void setTarget(int x, int y) {
    target.x = (float)x;
    target.y = (float)y;
}

void findTarget() {
    //WILL CHANGE LATER
    setTarget(200, 400);
}

float findAngle(pos_t* myPose) {
    float dx = target.x - myPose->x;
    float dy = target.y - myPose->y;
    return std::atan2f(dx, dy) / D_TO_R;
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
    float dt = float(now - lastTime)/MICRO_TO_SEC;
    lastTime = now;

    pos_vector_t myPOS = getMyPOS();

    switch (State) {
        case IDLE:
            {
            rotate.reset();
            translate.reset();

            //if we are too close to target choose a new one
            float dx = target.x - myPOS.pos.x;
            float dy = target.y - myPOS.pos.y;
            float dist = std::sqrt(dx*dx + dy*dy);

            State = (dist <= translate.tolerance) ? DONE : TURNING;
            break;
        }
        case TURNING:
            if (rotatePID(&myPOS, dt, rotate)) {setDriveSpeed(0,0); State = MOVEING;}
            break;
        case MOVEING:
            if (translatePID(&(myPOS.pos), dt, translate)) {setDriveSpeed(0,0); State = DONE;}
            break;
        case DONE:
            findTarget();
            State = IDLE;
            break;
    }
}