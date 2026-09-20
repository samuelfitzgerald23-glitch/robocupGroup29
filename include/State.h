#ifndef STATE_H
#define STATE_H

#include "moving.h"

enum robotState {
    CHECK,   
    FIND_TARGET,
    PATHFIND,
    MOVE,
    DETERMINE_ACTION,
    PICKUP_SEQUENCE,
    DROP_PAYLOAD
};

void StateMachineTask();

#endif //STATE_H