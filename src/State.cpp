#include "State.h"

IntervalTimer TimeoutCheck;

static int lastTime = 0;

static robotState CurrentState = CHECK;

static int8_t goal_type = 0;
static pos_t goal_pos = {0,0};

static pos_t microSteps[WAYPOINT_COUNT];

static void state_timeout() {
    //if a timeout occurs switch state to idle to start sequence again
    TimeoutCheck.end();
    CurrentState = CHECK;
}

void StateMachineTask() {
    int now = micros();
    float dt = float(now - lastTime)/MICRO_TO_SEC;
    lastTime = now;

    switch (CurrentState) {
        case CHECK:
            //check fornumber of weights and stuff idk
            CurrentState = FIND_TARGET;
            break;
        case FIND_TARGET:
            CurrentState = PATHFIND;
            break;
        case PATHFIND:  {
            int start_x, start_y, goal_x, goal_y;
            pos_t currentPos = getMyPOS().pos;

            worldToGrid(currentPos.x, currentPos.y, start_x, start_y);
            worldToGrid(goal_pos.x, goal_pos.y, goal_x, goal_y);

            if (findPathWaypoints({(int16_t)start_x, (int16_t)start_y}, {(int16_t)goal_x, (int16_t)goal_y}, microSteps)) {
                setPath(&microSteps[0]);
                CurrentState = MOVE;
            }
            break;
        }
        case MOVE:
            movementTask(dt);
            if(getPathStatus()) CurrentState = DETERMINE_ACTION;
            break;
        case DETERMINE_ACTION: {
            if(goal_type == static_cast<int8_t>(WEIGHT)) {
                CurrentState = PICKUP_SEQUENCE;
                //start a ten seccond timer to pick up the weight
                TimeoutCheck.begin(state_timeout, 10*MICRO_TO_SEC);
            } else if (goal_pos.x == 0 && goal_pos.y == 0) {
                CurrentState = DROP_PAYLOAD;
                //start a timer to drop the weights off
                TimeoutCheck.begin(state_timeout, 2*MICRO_TO_SEC);
            } else {
                CurrentState = CHECK;
            }
            break;
        }
        case PICKUP_SEQUENCE:
            break;
        case DROP_PAYLOAD:
            break;
        default:
            CurrentState = CHECK;
            break;
    }
}