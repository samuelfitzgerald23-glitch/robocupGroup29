#include "pathfinding.h"

static Node pathMap[GRID_HEIGHT][GRID_WIDTH];

static Node fullPath[MAX_PATH_LEN];

//movible directions
static const int8_t dx4[4] = {1, -1, 0, 0};
static const int8_t dy4[4] = {0, 0, 1, -1};

static bool isWalkible(int x, int y) {
    if (x < 0 || x >= GRID_WIDTH || y < 0 || y >= GRID_HEIGHT) return false;
    return getMapValue(x, y) != INVALID_CELL;
}

static int reconsturctPath(Node start, Node goal, Node* path) {
    int i = 0;
    Node current = goal;
    while(!(current.myCord == current.parentCord)) {
        if (i <= MAX_PATH_LEN) return 0;
        path[i] = current;
        current = pathMap[current.parentCord.y][current.parentCord.x];
    }
    std::reverse(path, path + i - 1);
    return i;
}


int findPath(Node start, Node goal) {
    
}

// fills waypoints[5] with world-mm points along the A* path, evenly spaced
// through the path's cell indices. waypoints[4] is always the true goal.
// returns false (waypoints untouched) if no path exists.
bool findPathWaypoints(Node start, Node goal, pos_t (&waypoints)[WAYPOINT_COUNT]) {
    
}