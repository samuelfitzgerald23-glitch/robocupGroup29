#include "State.h"

IntervalTimer TimeoutCheck;

static int lastTime = 0;

static robotState CurrentState = STARTUP;

static int8_t goal_type = 0;
static pos_t goal_pos = {1000,1000};

static pos_t microSteps[MAX_WAYPOINT_COUNT] = {{0,0},{0,250},{250,250},{250,0},{0,0}};

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
        case STARTUP:
            setPath(&microSteps[0], 5);
            CurrentState = MOVE;
            break;
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

            bool startValid = worldToGrid(currentPos.x, currentPos.y, start_x, start_y);
            bool goalValid = worldToGrid(goal_pos.x, goal_pos.y, goal_x, goal_y);

            if (!startValid || !goalValid) {
                break;  // don't pathfind from/to a clamped, not-really-there position
            }

            int targetCount = findPathWaypoints({{(uint8_t)start_x, (uint8_t)start_y}}, {{(uint8_t)goal_x, (uint8_t)goal_y}}, microSteps);

            if (targetCount > 0) {
                setPath(&microSteps[0], targetCount);
                CurrentState = MOVE;
            }
            break;
        }
        case MOVE:
            movementTask(dt);
            if(getPathStatus()) {
                if(getPathBlockedStatus()) {
                    CurrentState = PATHFIND;
                } else {
                    CurrentState = DETERMINE_ACTION;
                }
            }
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