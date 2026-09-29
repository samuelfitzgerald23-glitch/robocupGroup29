#include "target_finding.h"

#define BOX_X_MIN 12
#define BOX_Y_MIN 12
#define BOX_X_MAX (GRID_WIDTH-BOX_X_MIN)
#define BOX_Y_MAX (GRID_HEIGHT-BOX_Y_MIN)

static GridPos home = {X_ZERO,Y_ZERO};

//FIX: goals that failed to plan are remembered (oldest drop off) so the robot
//doesn't keep choosing the same unreachable "unexplored" cell. Only the single
//previous goal used to be excluded, so it could ping-pong between two bad ones.
#define UNREACHABLE_MEMORY 16
static GridPos unreachable[UNREACHABLE_MEMORY];
static int unreachableCount = 0;
static int unreachableNext = 0;

void markTargetUnreachable(GridPos p) {
    unreachable[unreachableNext] = p;
    unreachableNext = (unreachableNext + 1) % UNREACHABLE_MEMORY;
    if (unreachableCount < UNREACHABLE_MEMORY) unreachableCount++;
}

static bool isUnreachable(GridPos p) {
    for (int k = 0; k < unreachableCount; k++) {
        if (unreachable[k] == p) return true;
    }
    return false;
}

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
    bool foundCandidate = false;    //FIX: see end of function

    for (int i = x_min; i < x_max; i++) {
        for (int j = y_min; j < y_max; j++) {
            GridPos current = {i, j};
            if (current == prevGrid || current == robotGrid || current == home) continue;
            if (isUnreachable(current)) continue;
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
            bool bestSoFarIsUnexplored = foundCandidate &&
                (getMapValue(current_target.x, current_target.y) == static_cast<int8_t>(UNDEFINED_CELL));

            //is current best is not unexplored an unexplored cell will overwrite it
            //between cells in the same category distance wins

            bool better = (isUnexplored && !bestSoFarIsUnexplored) ||
                        (isUnexplored == bestSoFarIsUnexplored && dist <= cost);

            if (better || !foundCandidate) {
                current_target = current;
                cost = dist;
                foundCandidate = true;
            }
        }
    }
    target_type = NO_ACTION;

    //FIX: if nothing in the box qualified, this used to return the robot's own
    //cell -> start == goal -> "No path" forever. Head for the map centre instead.
    if (!foundCandidate) {
        current_target = {GRID_WIDTH/2, GRID_HEIGHT/2};
    }

    pos_t targetWorld;
    gridToWorld(current_target.x, current_target.y, targetWorld.x, targetWorld.y);

    return targetWorld;
}