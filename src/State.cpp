#include "State.h"
#include "LED_array.h"

IntervalTimer TimeoutCheck;

static int lastTime = 0;

static robotState CurrentState = STARTUP;

static int8_t goal_type = 0;
static pos_t goal_pos = {1000,1000};
static pos_t prev_target = goal_pos;

static pos_t microSteps[MAX_WAYPOINT_COUNT] = {{0,0},{0,200},{200,200},{200,0},{0,0}};

static void state_timeout() {
    //if a timeout occurs switch state to idle to start sequence again
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
            BLUETOOTH_SERIAL.print("G, "); BLUETOOTH_SERIAL.print(goal_pos.x); BLUETOOTH_SERIAL.print(", "); BLUETOOTH_SERIAL.print(goal_pos.y);
            CurrentState = PATHFIND;
            break;
        case CHECK:
            //check fornumber of weights and stuff idk
            CurrentState = FIND_TARGET;
            break;
        case FIND_TARGET:
            {
            pos_t prev_target = goal_pos;
            int cur_gx, cur_gy;
            worldToGrid(currentPos.x, currentPos.y, cur_gx, cur_gy);
            bool found = false;
            float cost = 0;

            for (int y = 0; y < GRID_HEIGHT && !found; y++) {
                for (int x = 0; x < GRID_WIDTH && !found; x++) {
                    if (getMapValue(x, y) == static_cast<int8_t>(WEIGHT)) {
                        gridToWorld(x, y, goal_pos.x, goal_pos.y);
                        goal_type = static_cast<int8_t>(WEIGHT);
                        found = true;
                        break;
                    }
                    if (getMapValue(x, y) == static_cast<int8_t>(UNDEFINED_CELL)) {
                        float dx = x - cur_gx;
                        float dy = y - cur_gy;
                        float temp = std::sqrt(dx*dx + dy*dy);
                        if (temp > cost) {
                            gridToWorld(x, y, goal_pos.x, goal_pos.y);
                            goal_type = static_cast<int8_t>(UNDEFINED_CELL);
                            cost = temp;
                        }
                    }
                }
            }

            if ((goal_pos.x == prev_target.x && goal_pos.y == prev_target.y) || !found) {
                goal_pos = {0,0};
            }

            prev_target = goal_pos;

            BLUETOOTH_SERIAL.print("G, "); BLUETOOTH_SERIAL.print(goal_pos.x); BLUETOOTH_SERIAL.print(", "); BLUETOOTH_SERIAL.print(goal_pos.y);
            CurrentState = PATHFIND;
            break;
        }
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