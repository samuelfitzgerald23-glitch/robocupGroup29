#ifndef PATHFINDING_H
#define PATHFINDING_H

#include "map.h"

#define ROOT_TWO 1.414f

#define MAX_PATH_LEN GRID_HEIGHT * GRID_WIDTH

struct GridPos {
    int16_t x, y;
    bool operator==(const GridPos& o) const {return x==o.x && y==o.y;}
};

struct Node {
    float gScore, fScore;
    int16_t parent_x, parent_y;
    bool isOpen, isClosed;
};

int findPath(GridPos start, GridPos goal, GridPos* path, int maxPathLen);

bool findPathWaypoints(GridPos start, GridPos goal, pos_t (&waypoints)[WAYPOINT_COUNT]);

#endif //PATHFINDING_H