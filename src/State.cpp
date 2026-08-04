#include "State.h"

IntervalTimer TimeoutCheck;

static int lastTime = 0;

static robotState CurrentState = MOVE;

void StateMachineTask() {
    int now = micros();
    float dt = float(now - lastTime)/MICRO_TO_SEC;
    lastTime = now;

    switch (CurrentState) {
        case IDLE:
            break;
        case FIND_TARGET:
            break;
        case PATHFIND:
            break;
        case MOVE:
            movementTask(dt);
            //if(getPathStatus()) CurrentState = DETERMINE_ACTION;
            break;
        case DETERMINE_ACTION:
            break;
        case PICKUP_SEQUENCE:
            break;
        case KEEP_REJECT:
            break;
        case DROP_PAYLOAD:
            break;
        default:
            break;
    }
}