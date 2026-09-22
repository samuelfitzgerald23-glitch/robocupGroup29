#ifndef STATE_H
#define STATE_H

#include "moving.h"
#include "motor.h"
#include "pathfinding.h"

enum robotState {
    STARTUP = -1,
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