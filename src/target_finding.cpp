#include "target_finding.h"

#define BOX_X_MIN 12
#define BOX_Y_MIN 12
#define BOX_X_MAX (GRID_WIDTH-BOX_X_MIN)
#define BOX_Y_MAX (GRID_HEIGHT-BOX_Y_MIN)

static GridPos home = {X_ZERO,Y_ZERO};

pos_t find_target(pos_t myPos, pos_t prev_pos, goal_t& target_type) {
    //find the next target to pathfind toward

    if(get_num_weights() >= MAX_WEIGHTS_ONBOARD) {
        target_type = BASE_ACTION;
        pos_t homeWorld;
        gridToWorld(home.x, home.y, homeWorld.x, homeWorld.y);
        return homeWorld;
    }                                                               //if we have collected all weights the robot should return to base

    GridPos robotGrid, prevGrid;
    worldToGrid(myPos.x, myPos.y, robotGrid.x, robotGrid.y);        //robot pos on grid
    worldToGrid(prev_pos.x, prev_pos.y, prevGrid.x, prevGrid.y);    //previous target on grid

    //define section of map the robot looks for targets in
    int x_min = ((robotGrid.x - TARGET_GRID_SIZE) <= BOX_X_MIN) ? BOX_X_MIN : (robotGrid.x - TARGET_GRID_SIZE);
    int x_max = ((robotGrid.x + TARGET_GRID_SIZE) >= BOX_X_MAX) ? BOX_X_MAX : (robotGrid.x + TARGET_GRID_SIZE);
    int y_min = ((robotGrid.y - TARGET_GRID_SIZE) <= BOX_Y_MIN) ? BOX_Y_MIN : (robotGrid.y - TARGET_GRID_SIZE);
    int y_max = ((robotGrid.y + TARGET_GRID_SIZE) >= BOX_Y_MAX) ? BOX_Y_MAX : (robotGrid.y + TARGET_GRID_SIZE);

    float cost = 1e9f;

    GridPos current_target = robotGrid;

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