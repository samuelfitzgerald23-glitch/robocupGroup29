#include "pathfinding.h"

static Node pathMap[GRID_HEIGHT][GRID_WIDTH];

static Node fullPath[MAX_PATH_LEN];

//movible directions
static const int8_t dx4[4] = {1, -1, 0, 0};
static const int8_t dy4[4] = {0, 0, 1, -1};

struct Queue {
    GridPos data[MAX_PATH_LEN];
    int head = 0, tail = 0;
    bool empty() const { return head == tail; }
    void push(GridPos p) { data[tail++] = p; }
    GridPos pop() { return data[head++]; }
};

static bool isWalkible(int x, int y) {
    if (x < 0 || x >= GRID_WIDTH || y < 0 || y >= GRID_HEIGHT) return false;
    return getMapValue(x, y) != INVALID_CELL;
}

static void clearMap() {
    for (uint8_t i = 0; i < GRID_HEIGHT; i++) {
        for (uint8_t j = 0; j < GRID_WIDTH; j++) {
            pathMap[i][j] = {{j,i}, {j,i}, 0};
        }
    }
}

static int reconsturctPath(Node start, Node goal, Node* path) {
    int i = 0;
    Node current = pathMap[goal.myCord.y][goal.myCord.x];  // the searched result, not the raw parameter
    while(!(current.myCord == current.parentCord)) {
        if (i >= MAX_PATH_LEN) return 0;
        path[i] = current;
        current = pathMap[current.parentCord.y][current.parentCord.x];
        i++;
    }
    std::reverse(path, path + i);
    return i;
}


int findPath(Node start, Node goal) {
    clearMap();

    static Queue q;
    q.head = q.tail = 0;

    GridPos s = start.myCord;
    pathMap[s.y][s.x].parentCord = s;   //sets loop terminator
    pathMap[s.y][s.x].visited = 1;
    q.push(s);
    
    bool found = 0;
    while(!q.empty()) {
        GridPos current = q.pop();
        if (current == goal.myCord) {found = 1; break;}

        for (int d = 0; d < 4; d++) {
            uint8_t dx = current.x + dx4[d];
            uint8_t dy = current.y + dy4[d];
            //check if we can go here or if weve already checked here
            if(!isWalkible(dx, dy)) continue;
            if(pathMap[dy][dx].visited) continue;

            pathMap[dy][dx].parentCord = current;
            pathMap[dy][dx].visited = 1;
            q.push({dx, dy});
        }
    }
    if(!found) return 0;
    return reconsturctPath(start, goal, fullPath);

}

// fills waypoints[5] with world-mm points along the A* path, evenly spaced
// through the path's cell indices. waypoints[4] is always the true goal.
// returns false (waypoints untouched) if no path exists.
bool findPathWaypoints(Node start, Node goal, pos_t (&waypoints)[WAYPOINT_COUNT]) {
    int pathlength = findPath(start, goal);
    if(pathlength == 0) return 0;

    for(int i = 0; i < WAYPOINT_COUNT; i++) {
        int srcIndex = (int)roundf((float)i * (pathlength - 1) / (WAYPOINT_COUNT - 1));
        if(srcIndex >= pathlength) srcIndex = pathlength - 1;

        float wx, wy;
        gridToWorld(fullPath[srcIndex].myCord.x, fullPath[srcIndex].myCord.y, wx, wy);
        waypoints[i] = {wx, wy};
    }
    return 1;
}