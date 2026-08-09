#ifndef PATHFINDING_H
#define PATHFINDING_H

#include "map.h"

#define ROOT_TWO 1.414f

#define MAX_PATH_LEN GRID_HEIGHT * GRID_WIDTH

struct GridPos {
    uint16_t x, y;
    bool operator==(const GridPos& o) const {return x==o.x && y==o.y;};
};

struct Node {
    GridPos myCord;
    GridPos parentCord;
    bool isWalkible;
};

#endif //PATHFINDING_H