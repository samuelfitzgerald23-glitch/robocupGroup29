#include "State.h"
#include "gate.h"
#include "LED_array.h"
#include "reactive.h" 

#define RANDOM_SIZE 500

IntervalTimer TimeoutCheck;

static uint32_t lastTime = 0;
static int pathfindFailures = 0;

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
    #if USE_REACTIVE_CONTROL
        reactiveTask();
        return;                 // skip the pathfinding state machine below
    #endif
    pos_t currentPos = getMyPOS().pos;
    uint32_t now = micros();
    float dt = float(now - lastTime)/MICRO_TO_SEC;
    lastTime = now;

    BLUETOOTH_SERIAL.print("S, "); BLUETOOTH_SERIAL.println(CurrentState);

    switch (CurrentState) {
        case STARTUP:
            setLEDColor(WRITIBLE_LED, GREEN);
            gridToWorld(GRID_WIDTH/2, GRID_HEIGHT/2, goal_pos.x, goal_pos.y);
            BLUETOOTH_SERIAL.print("G, "); BLUETOOTH_SERIAL.print(goal_pos.x); BLUETOOTH_SERIAL.print(", "); BLUETOOTH_SERIAL.println(goal_pos.y);
            CurrentState = PATHFIND;
            break;
        case CHECK:
            //check fornumber of weights and stuff idk
            CurrentState = FIND_TARGET;
            break;
        case FIND_TARGET:
            goal_pos = find_target(currentPos, goal_pos, goal_type);
            if (pathfindFailures > 5) {
                    pathfindFailures = 0;
                    // force a new goal
                    gridToWorld(GRID_WIDTH/2, GRID_HEIGHT/2, goal_pos.x, goal_pos.y);
                }
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
            } else {
                pathfindFailures++;
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
                //safety net: bail out if the gate never finishes (jam), LONGER
                //than the gate's 10 s hold so it only fires on a fault.
                TimeoutCheck.begin(state_timeout, 25*MICRO_TO_SEC);
            } else {
                CurrentState = CHECK;
            }
            break;
        }
        case PICKUP_SEQUENCE:
            break;

        case DROP_PAYLOAD:
            if (gateDrop()) {          // shakes + opens/holds/closes the gate
                TimeoutCheck.end();
                CurrentState = CHECK;
            }
            break;
        
        default:
            CurrentState = CHECK;
            break;
    }
}

robotState getRobotState() {
    return CurrentState;
}
