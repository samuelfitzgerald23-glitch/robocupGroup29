#include "pathfinding.h"

static Node nodes[GRID_HEIGHT][GRID_WIDTH];

//movible directions
static const int8_t dx8[8] = {1, -1, 0, 0, 1, 1, -1, -1};
static const int8_t dy8[8] = {0, 0, 1, -1, 1, -1, 1, -1};

static float heuristic(GridPos a, GridPos b) {
    //straight line dist between cells
    float dx = a.x - b.x, dy = a.y - b.y;
    return std::sqrt(dx*dx + dy*dy);
}

static bool isWalkible(int x, int y) {
    if (x < 0 || x >= GRID_WIDTH || y < 0 || y >= GRID_HEIGHT) return false;
    return getMapValue(x, y) != INVALID_CELL;
}

static void clearNodes() {
    //clear the pathfinding map for a new run
    for (int y = 0; y < GRID_HEIGHT; y++)
        for (int x = 0; x < GRID_WIDTH; x++)
            nodes[y][x] = {0, 0, -1, -1, false, false};
}

// simple O(n) scan for lowest fScore in the open set - fine at this grid size
// (40x80 = 3200 cells); swap for a binary heap if you ever scale up much further
static bool popLowestOpen(GridPos& out) {
    float best = 1e9f;
    bool found = 0;

    for(int y = 0; y < GRID_HEIGHT; y++) {
        for (int x = 0; x < GRID_WIDTH; x++) {
            if (nodes[y][x].isOpen && nodes[y][x].fScore < best) {
                best = nodes[y][x].fScore;
                out = {(int16_t)x, (int16_t)y};
                found = 1;
            }
        }
    }
    if (found) nodes[out.y][out.x].isOpen = 0;
    return found;
}


int findPath(GridPos start, GridPos goal, GridPos* path, int maxPathLen) {
    clearNodes();

    if (!isWalkible(goal.x, goal.y)) return 0;

    nodes[start.y][start.x].gScore = 0;
    nodes[start.y][start.x].fScore = heuristic(start, goal);
    nodes[start.y][start.x].isOpen = 1;

    GridPos current;
    while (popLowestOpen(current)) {

        if(current == goal) {
            int count = 0;
            GridPos walk = current;
            while(!(walk == start) && count < maxPathLen) {
                path[count++] = walk;
                Node& n = nodes[walk.y][walk.x];
                walk = {n.parent_x, n.parent_y};
            }
            std::reverse(path, path + count);
            return count;
        }

        nodes[current.y][current.x].isClosed = 1;

        for (int i = 0; i < 8; i++) {
            int nextx = current.x + dx8[i], nexty = current.y + dy8[i];
            if(!isWalkible(nextx, nexty)) continue;
            if(nodes[nexty][nextx].isClosed) continue;

            float stepCost = (dx8[i] != 0 && dy8[i] != 0) ? ROOT_TWO : 1.0f;
            float next_g = nodes[current.y][current.x].gScore + stepCost;

            if (!nodes[nexty][nextx].isOpen || next_g < nodes[nexty][nextx].gScore) {
                nodes[nexty][nextx].gScore = next_g;
                nodes[nexty][nextx].fScore = next_g + heuristic({(int16_t)nextx, (int16_t)nexty}, goal);
                nodes[nexty][nextx].parent_x = current.x;
                nodes[nexty][nextx].parent_y = current.y;
                nodes[nexty][nextx].isOpen = true;
            }
        }
    }
    //if all paths are exhausted
    return 0;
}