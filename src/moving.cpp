#include "moving.h"

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

static float P = 1;
static float I = 0;
static float D = 0;

static pos_t target;

void setTarget(int x, int y) {
    target.x = (float)x;
    target.y = (float)y;
}

float findDistence(pos_vector_t* myPose) {
    //find out error between our current Pose and our target Pose
    float myX = myPose->pos.x;
    float myY = myPose->pos.y;

    float dx = target.x - myX;
    float dy = target.y - myY;

    return std::sqrt((dx * dx) + (dy * dy));
}