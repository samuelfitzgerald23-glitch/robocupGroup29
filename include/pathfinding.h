#ifndef PATHFINDING_H
#define PATHFINDING_H

#include "map.h"

#define ROOT_TWO 1.414f

#define MAX_PATH_LEN (GRID_HEIGHT * GRID_WIDTH)   //FIX: brackets

struct Node {
    GridPos myCord;
    GridPos parentCord;
    bool visited;
};

int findPath(Node start, Node goal);

int findPathWaypoints(Node start, Node goal, pos_t* waypoints);

//true if the last findPathWaypoints() ran out of waypoint slots before the goal
bool lastPathTruncated();

#endif //PATHFINDING_H