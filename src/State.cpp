#include "State.h"
#include "LED_array.h"

IntervalTimer TimeoutCheck;

static int lastTime = 0;

static robotState CurrentState = STARTUP;

static goal_t goal_type = NO_ACTION;
static pos_t goal_pos;

static pos_t microSteps[MAX_WAYPOINT_COUNT];

static void state_timeout() {
    //if a timeout occurs switch state to idle to start sequence again
    setDriveSpeed(0,0);
    TimeoutCheck.end();
    CurrentState = CHECK;
}

void StateMachineTask() {
    pos_t currentPos = getMyPOS().pos;
    int now = micros();
    float dt = float(now - lastTime)/MICRO_TO_SEC;
    lastTime = now;

    BLUETOOTH_SERIAL.print("S, "); BLUETOOTH_SERIAL.println(CurrentState);

    switch (CurrentState) {
        case STARTUP:
            setLEDColor(WRITIBLE_LED, GREEN);
            setDriveSpeed(250, -250);
            TimeoutCheck.begin(state_timeout, (int)(1.5*MICRO_TO_SEC));
            CurrentState = CHECK;
            break;
        case CHECK:
            //check fornumber of weights and stuff idk
            CurrentState = FIND_TARGET;
            break;
        case FIND_TARGET:
            goal_pos = find_target(currentPos, goal_pos, goal_type);
            BLUETOOTH_SERIAL.print("G, "); BLUETOOTH_SERIAL.print(goal_pos.x); BLUETOOTH_SERIAL.print(", "); BLUETOOTH_SERIAL.println(goal_pos.y);
            CurrentState = PATHFIND;
            break;
        case PATHFIND:  {
            setLEDColor(WRITIBLE_LED, RED);
            int start_x, start_y, goal_x, goal_y;

            bool startValid = worldToGrid(currentPos.x, currentPos.y, start_x, start_y);
            bool goalValid = worldToGrid(goal_pos.x, goal_pos.y, goal_x, goal_y);

            if (!startValid || !goalValid) {
                CurrentState = CHECK;
                break;
            }

            int targetCount = findPathWaypoints({{(uint8_t)start_x, (uint8_t)start_y}}, {{(uint8_t)goal_x, (uint8_t)goal_y}}, microSteps);

            if (targetCount > 0) {
                setPath(&microSteps[0], targetCount);
                CurrentState = MOVE;
            } if (targetCount == 0) {
                CurrentState = CHECK;
            }
            break;
        }
        case MOVE:
            setLEDColor(WRITIBLE_LED, WHITE);
            movementTask(dt);
            if (getPathStatus()) {
                CurrentState = getPathBlockedStatus() ? PATHFIND : DETERMINE_ACTION;
            }
            break;
        case DETERMINE_ACTION: {
            setLEDColor(WRITIBLE_LED, BLUE);
            if(goal_type == WEIGHT_ACTION) {
                CurrentState = PICKUP_SEQUENCE;
                setDriveSpeed(300,300);
                TimeoutCheck.begin(state_timeout, 1*MICRO_TO_SEC);
            } else if (goal_type == BASE_ACTION) {
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