#include "target_finding.h"

static GridPos home = {0,0};

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

    float cost = 100;

    GridPos current_target = {0, 0};

    for (int i = x_min; i < x_max; i++) {
        for (int j = y_min; j < y_max; j++) {
            GridPos current = {i, j};
            if (current == prevGrid || current == robotGrid || current == home) continue;
            if (!isWalkible(i, j, ROBOT_WIDTH)) continue;
            int value = getMapValue(i, j);
            if (value == static_cast<int8_t>(WEIGHT)) {
                target_type = WEIGHT_ACTION;
                pos_t targetWorld;
                gridToWorld(current.x, current.y, targetWorld.x, targetWorld.y);

                return targetWorld;
            }
            int dx = current.x - GRID_WIDTH/2;
            int dy = current.y - GRID_HEIGHT/2;
            float dist = std::sqrt(dx*dx + dy*dy);

            bool isUnexplored = (value == static_cast<int8_t>(UNDEFINED_CELL));
            bool bestSoFarIsUnexplored = (getMapValue(current_target.x, current_target.y) == static_cast<int8_t>(UNDEFINED_CELL));

            //is current best is not unexplored an unexplored cell will overwrite it
            //between cells in the same category distance wins

            bool better = (isUnexplored && !bestSoFarIsUnexplored) ||
                        (isUnexplored == bestSoFarIsUnexplored && dist <= cost);

            if (better) {
                current_target = current;
                cost = dist;
            }
        }
    }
    target_type = NO_ACTION;

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