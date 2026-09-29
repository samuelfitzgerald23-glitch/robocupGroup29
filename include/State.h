#ifndef STATE_H
#define STATE_H

#include "moving.h"
#include "motor.h"
#include "pathfinding.h"
#include "target_finding.h"

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
robotState getRobotState();

#endif //STATE_H
