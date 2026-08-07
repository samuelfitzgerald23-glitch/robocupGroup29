#ifndef PATHFINDING_H
#define PATHFINDING_H

#include "map.h"

#define ROOT_TWO 1.414f

struct GridPos {
    int16_t x, y;
    bool operator==(const GridPos& o) const {return x==o.x && y==o.y;}
};

struct Node {
    float gScore, fScore;
    int16_t parent_x, parent_y;
    bool isOpen, isClosed;
};

#endif //PATHFINDING_H