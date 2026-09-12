#ifndef PATHFINDING_H
#define PATHFINDING_H

#include "map.h"

#define ROOT_TWO 1.414f

#define MAX_PATH_LEN GRID_HEIGHT * GRID_WIDTH

struct GridPos {
    int x, y;
    bool operator==(const GridPos& o) const {return x==o.x && y==o.y;};
};

struct Node {
    GridPos myCord;
    GridPos parentCord;
    bool visited;
};

int findPath(Node start, Node goal);

int findPathWaypoints(Node start, Node goal, pos_t* waypoints);

#endif //PATHFINDING_H