#include "target_finding.h"

static float conv_kernel[KERNEL_SIZE][KERNEL_SIZE] = {
    {1/9,1/9,1/9},
    {1/9,1/9,1/9},
    {1/9,1/9,1/9}
};

static bool collector_full() {
    return getNumWeights() >= MAX_WEIGHTS_ONBOARD;
}

pos_t find_target(pos_t myPos, pos_t prev_pos) {
    //find the next target to pathfind toward

    if(collector_full) return {0,0};        //if we have collected all weights the robot should return to base

    int rx, ry, px, py;
    worldToGrid(myPos.x, myPos.y, rx, ry);          //robot pos on grid
    worldToGrid(prev_pos.x, prev_pos.y, px, py);    //previous target on grid
}