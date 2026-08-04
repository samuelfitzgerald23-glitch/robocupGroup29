#include "State.h"

IntervalTimer TimeoutCheck;

static int lastTime = 0;

static robotState CurrentState = MOVE;

static int8_t target_type;
static pos_t target_pos;

void StateMachineTask() {
    int now = micros();
    float dt = float(now - lastTime)/MICRO_TO_SEC;
    lastTime = now;

    switch (CurrentState) {
        case IDLE:
            CurrentState = FIND_TARGET;
            break;
        case FIND_TARGET:
            break;
        case PATHFIND:
            break;
        case MOVE:
            movementTask(dt);
            if(getPathStatus()) CurrentState = DETERMINE_ACTION;
            break;
        case DETERMINE_ACTION: {
            if(target_type == static_cast<int8_t>(WEIGHT)) {
                CurrentState = PICKUP_SEQUENCE;
                //start a ten seccond timer to pick up the weight
                TimeoutCheck.begin(state_timeout, 10*MICRO_TO_SEC);
            } else if (target_pos.x == 0 && target_pos.y == 0) {
                CurrentState = DROP_PAYLOAD;
                //start a timer to drop the weights off
                TimeoutCheck.begin(state_timeout, 2*MICRO_TO_SEC);
            } else {
                CurrentState = IDLE;
            }
            break;
        }
        case PICKUP_SEQUENCE:
            break;
        case KEEP_REJECT:
            break;
        case DROP_PAYLOAD:
            break;
        default:
            CurrentState = IDLE;
            break;
    }
}

static void state_timeout() {
    //if a timeout occurs switch state to idle to start sequence again
    TimeoutCheck.end();
    CurrentState = IDLE;
}