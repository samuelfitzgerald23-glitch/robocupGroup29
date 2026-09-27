#include "target_finding.h"

// static float conv_kernel[KERNEL_SIZE][KERNEL_SIZE] = {
//     {1/9,1/9,1/9},
//     {1/9,1/9,1/9},
//     {1/9,1/9,1/9}
// };

pos_t find_target(pos_t myPos, pos_t prev_pos, goal_t& target_type) {
    //find the next target to pathfind toward

    if(getNumWeights() >= MAX_WEIGHTS_ONBOARD) {
        target_type = BASE_ACTION;
        return {0,0};
    }                                                               //if we have collected all weights the robot should return to base

    GridPos robotGrid, prevGrid;
    worldToGrid(myPos.x, myPos.y, robotGrid.x, robotGrid.y);        //robot pos on grid
    worldToGrid(prev_pos.x, prev_pos.y, prevGrid.x, prevGrid.y);    //previous target on grid

    //define section of map the robot looks for targets in
    int x_min = ((robotGrid.x - TARGET_GRID_SIZE) <= 0) ? 0 : (robotGrid.x - TARGET_GRID_SIZE);
    int x_max = ((robotGrid.x + TARGET_GRID_SIZE) >= GRID_WIDTH) ? GRID_WIDTH : (robotGrid.x + TARGET_GRID_SIZE);
    int y_min = ((robotGrid.y - TARGET_GRID_SIZE) <= 0) ? 0 : (robotGrid.y - TARGET_GRID_SIZE);
    int y_max = ((robotGrid.y + TARGET_GRID_SIZE) >= GRID_HEIGHT) ? GRID_HEIGHT : (robotGrid.y + TARGET_GRID_SIZE);

    float priority;
    float dist;
    float cost = 0;

    GridPos current_target = {0, 0};

    for (size_t i = x_min; i < x_max; i++) {
        for (size_t j = y_min; j < y_max; j++) {
            GridPos current = {i, j};
            if (current == prevGrid || current == robotGrid) continue;
            if (!isWalkible(i, j, ROBOT_WIDTH)) continue;
            priority = getMapValue(i, j);
            int dx = current.x - robotGrid.x;
            int dy = current.y - robotGrid.y;
            dist = std::sqrt(dx*dx + dy*dy);

            if ((priority * dist) > cost) {
                current_target = current;
                cost = priority * dist;
            }
        }
    }
    if (priority >= 0) target_type = NO_ACTION;
    if (priority == static_cast<int8_t>(WEIGHT)) target_type = WEIGHT_ACTION;

    pos_t targetWorld;
    gridToWorld(current_target.x, current_target.y, targetWorld.x, targetWorld.y);

    return targetWorld;
}

/*
pros vs cons of doing a convolution on target map:

pros:
allows for frontier edge detection
gets a smoother transition between cells
due to smoothing cells around weights have higher prioritys, and cells around walls have lower priority

cons:
possibly take a while
hard to implament
*/