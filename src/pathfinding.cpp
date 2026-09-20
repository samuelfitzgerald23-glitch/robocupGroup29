#include "pathfinding.h"
#include <vector>

#define WALL_CELL_INFLATION 5       //number of cells out from the wall we check to avoid it

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


static int findPathAtInflation(Node start, Node goal, int inflation) {
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
            int dx = current.x + dx4[d];
            int dy = current.y + dy4[d];
            //check if we can go here or if weve already checked here
            if(!isWalkible(dx, dy, inflation)) continue;
            if(pathMap[dy][dx].visited) continue;

            pathMap[dy][dx].parentCord = current;
            pathMap[dy][dx].visited = 1;
            q.push({dx, dy});
        }
    }


    if(!found) return 0;
    return reconsturctPath(start, goal, fullPath);

}

int findPath(Node start, Node goal) {
    for (int inflation = WALL_CELL_INFLATION; inflation > ROBOT_WIDTH; inflation--) {
        if (findPathAtInflation(start, goal, inflation)) {
            if (inflation < WALL_CELL_INFLATION) {
                Serial.print("findPath: reduced inflation to ");
                Serial.print(inflation);
                Serial.println(" to find a route");
            }
            return reconsturctPath(start, goal, fullPath);
        }
    }
    return 0;  // no path even with zero inflation - genuinely blocked
}

int findPathWaypoints(Node start, Node goal, pos_t* waypoints) {
    int pathlength = findPath(start, goal);
    if(pathlength == 0) return 0;

    int count = 0;
    int dist = 0;
    float wx, wy;
    gridToWorld(fullPath[0].myCord.x, fullPath[0].myCord.y, wx, wy);
    waypoints[count++] = {wx, wy};

    for(int i = 1; i < pathlength && count < MAX_WAYPOINT_COUNT; i++) {
        dist += CELL_SIZE;
        bool isTurn = false;
        if (i < pathlength - 1) {
            int dx1 = fullPath[i].myCord.x - fullPath[i-1].myCord.x;
            int dy1 = fullPath[i].myCord.y - fullPath[i-1].myCord.y;
            int dx2 = fullPath[i+1].myCord.x - fullPath[i].myCord.x;
            int dy2 = fullPath[i+1].myCord.y - fullPath[i].myCord.y;
            isTurn = (dx1 != dx2) || (dy1 != dy2);
        }
        bool isGoal = (i == pathlength - 1);

        if (isGoal || isTurn || dist >= WAYPOINT_SPACING) {
            gridToWorld(fullPath[i].myCord.x, fullPath[i].myCord.y, wx, wy);
            waypoints[count++] = {wx, wy};
            dist = 0.0f;
        }
    }
    return count;
}