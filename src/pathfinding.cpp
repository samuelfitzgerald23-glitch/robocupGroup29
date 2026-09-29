#include "pathfinding.h"
#include <algorithm>

#define WALL_CELL_INFLATION 7       //number of cells out from the wall we check to avoid it

//FIX (corner base): within this many cells of the start or goal, the robot is
//allowed to be as close to walls as it already is there, instead of needing the
//full clearance. Home base is in a corner, so the start/goal can be right next
//to a wall - without this the planner found "No path" from (or back to) base.
#define ESCAPE_RADIUS 8             //cells (8 x 50 mm = 400 mm)

#define CELL_INDEX(x, y) ((int16_t)((y) * GRID_WIDTH + (x)))

//FIX: compact search state. The old Node pathMap + Node fullPath + GridPos queue
//used ~225 KB of RAM; this uses ~25 KB.
static int16_t parentIdx[MAX_PATH_LEN];      //-1 = not visited, start points to itself
static int16_t queueBuf[MAX_PATH_LEN];
static int16_t fullPath[MAX_PATH_LEN];       //cell indices, start excluded, goal last

//FIX: clearance[y][x] = square (Chebyshev) distance in cells to the nearest
//obstacle or map edge. A cell is walkable at inflation r when clearance > r.
//This is exactly the square check in isWalkible(), but computed once per plan
//instead of re-scanning (2r+1)^2 cells for every neighbour the BFS looks at.
static uint8_t clearance[GRID_HEIGHT][GRID_WIDTH];

static bool lastTruncated = false;

//movible directions
static const int8_t dx4[4] = {1, -1, 0, 0};
static const int8_t dy4[4] = {0, 0, 1, -1};

static inline uint8_t minU8(uint8_t a, int b) { return (b < a) ? (uint8_t)b : a; }

static void buildClearanceMap() {
    //initialise: obstacles 0, everything else = distance to the outside of the map
    for (int y = 0; y < GRID_HEIGHT; y++) {
        for (int x = 0; x < GRID_WIDTH; x++) {
            if (getMapValue(x, y) < CONFIDENCE_THRESHOLD) {
                clearance[y][x] = 0;
            } else {
                int border = std::min(std::min(x + 1, y + 1), std::min(GRID_WIDTH - x, GRID_HEIGHT - y));
                clearance[y][x] = (uint8_t)std::min(border, 255);
            }
        }
    }
    //forward pass (top-left neighbours)
    for (int y = 0; y < GRID_HEIGHT; y++) {
        for (int x = 0; x < GRID_WIDTH; x++) {
            uint8_t c = clearance[y][x];
            if (x > 0)                          c = minU8(c, clearance[y][x-1] + 1);
            if (y > 0) {
                                                c = minU8(c, clearance[y-1][x] + 1);
                if (x > 0)                      c = minU8(c, clearance[y-1][x-1] + 1);
                if (x < GRID_WIDTH - 1)         c = minU8(c, clearance[y-1][x+1] + 1);
            }
            clearance[y][x] = c;
        }
    }
    //backward pass (bottom-right neighbours)
    for (int y = GRID_HEIGHT - 1; y >= 0; y--) {
        for (int x = GRID_WIDTH - 1; x >= 0; x--) {
            uint8_t c = clearance[y][x];
            if (x < GRID_WIDTH - 1)             c = minU8(c, clearance[y][x+1] + 1);
            if (y < GRID_HEIGHT - 1) {
                                                c = minU8(c, clearance[y+1][x] + 1);
                if (x < GRID_WIDTH - 1)         c = minU8(c, clearance[y+1][x+1] + 1);
                if (x > 0)                      c = minU8(c, clearance[y+1][x-1] + 1);
            }
            clearance[y][x] = c;
        }
    }
}

static bool inGrid(GridPos p) {
    return p.x >= 0 && p.x < GRID_WIDTH && p.y >= 0 && p.y < GRID_HEIGHT;
}

static inline int chebyshev(int x0, int y0, int x1, int y1) {
    return std::max(abs(x1 - x0), abs(y1 - y0));
}

//clearance a cell must EXCEED to be walkable: normally `inflation`, but near the
//start/goal only "no worse than the start/goal cell itself" (and never an obstacle)
static inline int requiredClearance(int x, int y, GridPos s, GridPos g,
                                    int inflation, int startLimit, int goalLimit) {
    int r = inflation;
    if (chebyshev(x, y, s.x, s.y) <= ESCAPE_RADIUS) r = std::min(r, startLimit);
    if (chebyshev(x, y, g.x, g.y) <= ESCAPE_RADIUS) r = std::min(r, goalLimit);
    return r;
}

static int findPathAtInflation(GridPos s, GridPos g, int inflation) {
    int startLimit = std::min(inflation, std::max((int)clearance[s.y][s.x] - 1, 0));
    int goalLimit  = std::min(inflation, std::max((int)clearance[g.y][g.x] - 1, 0));

    //goal can never be reached (it is inside an obstacle) - don't bother searching
    if (clearance[g.y][g.x] <= requiredClearance(g.x, g.y, s, g, inflation, startLimit, goalLimit)) return 0;

    for (int i = 0; i < MAX_PATH_LEN; i++) parentIdx[i] = -1;

    int head = 0, tail = 0;
    int16_t startIdx = CELL_INDEX(s.x, s.y);
    int16_t goalIdx  = CELL_INDEX(g.x, g.y);
    parentIdx[startIdx] = startIdx;     //sets loop terminator
    queueBuf[tail++] = startIdx;

    bool found = false;
    while (head < tail) {
        int16_t cur = queueBuf[head++];
        if (cur == goalIdx) { found = true; break; }
        int cx = cur % GRID_WIDTH;
        int cy = cur / GRID_WIDTH;

        for (int d = 0; d < 4; d++) {
            int nx = cx + dx4[d];
            int ny = cy + dy4[d];
            if (nx < 0 || nx >= GRID_WIDTH || ny < 0 || ny >= GRID_HEIGHT) continue;
            //check if we can go here or if weve already checked here
            if (clearance[ny][nx] <= requiredClearance(nx, ny, s, g, inflation, startLimit, goalLimit)) continue;
            int16_t n = CELL_INDEX(nx, ny);
            if (parentIdx[n] != -1) continue;

            parentIdx[n] = cur;
            queueBuf[tail++] = n;
        }
    }

    if (!found) return 0;

    //reconstruct goal -> start, then reverse (start itself is not included)
    int len = 0;
    int16_t cur = goalIdx;
    while (cur != startIdx) {
        if (len >= MAX_PATH_LEN) return 0;
        fullPath[len++] = cur;
        cur = parentIdx[cur];
    }
    std::reverse(fullPath, fullPath + len);
    return len;
}

int findPath(Node start, Node goal) {
    if (!inGrid(start.myCord) || !inGrid(goal.myCord)) return 0;

    buildClearanceMap();

    for (int inflation = WALL_CELL_INFLATION; inflation >= ROBOT_WIDTH; inflation--) {
        int len = findPathAtInflation(start.myCord, goal.myCord, inflation);
        if (len) {
            BLUETOOTH_SERIAL.print("W, "); BLUETOOTH_SERIAL.println(len);
            return len;
        }
    }
    BLUETOOTH_SERIAL.print("W, "); BLUETOOTH_SERIAL.println("No path");
    return 0;  // no path even at minimum inflation - genuinely blocked
}

int findPathWaypoints(Node start, Node goal, pos_t* waypoints) {
    lastTruncated = false;
    float wx, wy;

    //FIX: start == goal used to return 0 ("no path"), which the state machine
    //treated as a failure forever. Return one waypoint: the goal cell centre.
    if (start.myCord == goal.myCord) {
        gridToWorld(goal.myCord.x, goal.myCord.y, wx, wy);
        waypoints[0] = {wx, wy};
        return 1;
    }

    int pathlength = findPath(start, goal);
    if (pathlength == 0) return 0;

    int count = 0;
    float dist = 0;
    int prevX = start.myCord.x, prevY = start.myCord.y;

    for (int i = 0; i < pathlength; i++) {
        int cx = fullPath[i] % GRID_WIDTH, cy = fullPath[i] / GRID_WIDTH;
        dist += CELL_SIZE;

        bool isGoal = (i == pathlength - 1);
        bool isTurn = false;
        if (!isGoal) {
            int nx = fullPath[i+1] % GRID_WIDTH, ny = fullPath[i+1] / GRID_WIDTH;
            isTurn = ((cx - prevX) != (nx - cx)) || ((cy - prevY) != (ny - cy));
        }
        prevX = cx; prevY = cy;

        //FIX: the first path cell is only 50 mm away. Always making it a waypoint
        //made the robot turn (sometimes sharply) towards a point it was nearly on.
        if (i == 0 && !isGoal) continue;

        if (isGoal || isTurn || dist >= WAYPOINT_SPACING) {
            //FIX: if we run out of waypoint slots, say so. The state machine used to
            //treat a cut-off path as "arrived at the goal".
            if (count >= MAX_WAYPOINT_COUNT) { lastTruncated = true; break; }
            gridToWorld(cx, cy, wx, wy);
            waypoints[count++] = {wx, wy};
            dist = 0.0f;
        }
    }
    return count;
}

bool lastPathTruncated() {
    return lastTruncated;
}
