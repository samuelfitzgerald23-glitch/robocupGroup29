#ifndef STATE_H
#define STATE_H

#include "moving.h"

enum robotState {
    IDLE,   
    FIND_TARGET,
    PATHFIND,
    MOVE,
    DETERMINE_ACTION,
    PICKUP_SEQUENCE,
    KEEP_REJECT,
    DROP_PAYLOAD
};

void StateMachineTask();

#endif //STATE_H