#include "State.h"
#include "gate.h"
#include "LED_array.h"
#include "reactive.h"

#define RANDOM_SIZE 500

#define PICKUP_TIME_MS        1000      //drive forward this long to scoop a weight
#define DROP_TIMEOUT_MS       25000     //safety net if the gate never finishes (jam)
#define STARTUP_MAP_UPDATES   10        //filter updates (100 ms each) to wait before the first plan
#define MAX_REPLANS           3         //blocked/truncated replans before choosing a new goal

static uint32_t lastTime = 0;
static int pathfindFailures = 0;
static int replans = 0;

static robotState CurrentState = STARTUP;
static robotState lastLoggedState = DROP_PAYLOAD;

static goal_t goal_type = NO_ACTION;
static pos_t goal_pos;

static pos_t microSteps[MAX_WAYPOINT_COUNT];

//FIX: timeouts are checked with millis() inside the task. The IntervalTimer ISR
//used to set CurrentState from interrupt context (not volatile -> race).
static uint32_t timeoutStart = 0;
static uint32_t timeoutLength = 0;

static void startTimeout(uint32_t ms) {
    timeoutStart = millis();
    timeoutLength = ms;
}

static bool timedOut() {
    return (millis() - timeoutStart) >= timeoutLength;
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

    //FIX: only log on change (was sent every 5 ms)
    if (CurrentState != lastLoggedState && BLUETOOTH_SERIAL.availableForWrite() >= 8) {
        BLUETOOTH_SERIAL.print("S, "); BLUETOOTH_SERIAL.println(CurrentState);
        lastLoggedState = CurrentState;
    }

    switch (CurrentState) {
        case STARTUP:
            setLEDColor(WRITIBLE_LED, GREEN);
            //FIX (corner base): let the sensors map the nearby walls (~1 s) before
            //planning the first route, so it isn't planned on an empty map
            if (getFilterUpdateCount() < STARTUP_MAP_UPDATES) break;
            gridToWorld(GRID_WIDTH/2, GRID_HEIGHT/2, goal_pos.x, goal_pos.y);
            BLUETOOTH_SERIAL.print("G, "); BLUETOOTH_SERIAL.print(goal_pos.x); BLUETOOTH_SERIAL.print(", "); BLUETOOTH_SERIAL.println(goal_pos.y);
            replans = 0;
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
                //FIX: the forced centre goal kept the old goal_type, so a failed
                //trip home ended with DROP_PAYLOAD in the middle of the arena
                goal_type = NO_ACTION;
            }
            BLUETOOTH_SERIAL.print("G, "); BLUETOOTH_SERIAL.print(goal_pos.x); BLUETOOTH_SERIAL.print(", "); BLUETOOTH_SERIAL.println(goal_pos.y);
            replans = 0;
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

            int targetCount = findPathWaypoints({{start_x, start_y}}, {{goal_x, goal_y}}, microSteps);

            if (targetCount > 0) {
                setPath(&microSteps[0], targetCount);
                CurrentState = MOVE;
            } else {
                //FIX: this used to stay in PATHFIND and retry the same impossible goal
                //every 5 ms forever (the "> 5 failures" reset is in FIND_TARGET, which
                //was never reached). Remember the goal as unreachable and pick another.
                pathfindFailures++;
                markTargetUnreachable({goal_x, goal_y});
                CurrentState = FIND_TARGET;
            }
            break;
        }
        case MOVE:
            setLEDColor(WRITIBLE_LED, WHITE);
            movementTask(dt);
            if (getPathStatus()) {
                //FIX: replan if the path was blocked OR was cut short (too many
                //waypoints) - both used to be treated as "arrived at the goal"
                bool needReplan = getPathBlockedStatus() || lastPathTruncated();
                if (needReplan) {
                    replans++;
                    CurrentState = (replans > MAX_REPLANS) ? FIND_TARGET : PATHFIND;
                } else {
                    pathfindFailures = 0;
                    CurrentState = DETERMINE_ACTION;
                }
            }
            break;
        case DETERMINE_ACTION: {
            setLEDColor(WRITIBLE_LED, BLUE);
            if(goal_type == WEIGHT_ACTION) {
                CurrentState = PICKUP_SEQUENCE;
                setDriveSpeed(300,300);
                startTimeout(PICKUP_TIME_MS);
            } else if (goal_type == BASE_ACTION) {
                CurrentState = DROP_PAYLOAD;
                gateStartDrop();                //FIX: always start the drop from the beginning
                //safety net: bail out if the gate never finishes (jam), LONGER
                //than the gate's 20 s sequence so it only fires on a fault.
                startTimeout(DROP_TIMEOUT_MS);
            } else {
                CurrentState = CHECK;
            }
            break;
        }
        case PICKUP_SEQUENCE:
            if (timedOut()) {
                setDriveSpeed(0,0);
                CurrentState = CHECK;
            }
            break;

        case DROP_PAYLOAD:
            if (gateDrop()) {          // shakes + opens/holds/closes the gate
                //FIX: the weight count was never reset, so after the first drop
                //find_target() sent the robot home forever
                reset_num_weights();
                CurrentState = CHECK;
            } else if (timedOut()) {
                setDriveSpeed(0,0);
                reset_num_weights();
                gateStartDrop();
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
