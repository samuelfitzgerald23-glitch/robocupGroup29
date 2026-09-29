#include "moving.h"
#include "motor.h"
#include <string>
#include <Arduino.h>

/*
TODO:
- Tune PID controllers (values below are starting points)
*/

#define MOVE_TIME_LIMIT_MS   3000   //max time to drive to one waypoint
#define TURN_TIME_LIMIT_MS   4000   //FIX: TURNING had no timeout at all
#define WAYPOINT_PATIENCE    3      //number of times robot can fail to reach a waypoint

#define ROTATE_SETTLE_COUNT  6      //FIX: must be inside tolerance this many calls in a row (~30 ms)
#define STEER_MIN_DIST       100.0f //stop steering this close to the target (direction gets noisy)
#define RETURN_HEADING_ERR   35.0f  //if heading is this far off while driving, stop and re-turn
#define MIN_DRIVE_SPEED      60     //stop the ESC deadband stalling us just short of the target

static moveingState State = IDLE;
static moveingState lastLoggedState = DONE;
static uint32_t stateStart = 0;
static int settleCount = 0;

//FIX: output limits set to 350 = the real motor range (1500 +/- 350 us).
//translate kp was 10 -> saturated for any error > 43 mm (full speed bang-bang).
static PID_t rotate(10.5, 0.1, 0.5, 375, 350, 5);
static PID_t translate(2.5, 0, 0.2, 0, 350, 50);
static PID_t steer(4.0, 0, 0.2, 0, 150, 0);     //FIX: new - heading correction while driving

static pos_t targetlist[MAX_WAYPOINT_COUNT];
static int targetCount = 0;
static int targetIndex = 0;
bool pathfinished = 0;
bool pathBlocked = 0;

static int path_failures = 0;

static pos_t currentTarget = {0,0};

//FIX: the IntervalTimer callback used to change State from an interrupt
//(State isn't volatile, and the main loop could re-start the motors right after
//the ISR stopped them). Timeouts are now checked with millis() in movementTask.
static void enterState(moveingState s) {
    State = s;
    stateStart = millis();
}

static void stopMotors() {
    setDriveSpeed(0, 0);
}

static bool pathSegmentBlocked(pos_t from, pos_t to) {
    int x0, y0, x1, y1;
    if(!worldToGrid(from.x, from.y, x0, y0) || !worldToGrid(to.x, to.y, x1, y1)) return true;

    //FIX (corner base): normally every cell on the segment needs ROBOT_WIDTH-1
    //cells of clearance. But when the robot (or its target) is already closer to a
    //wall than that - e.g. leaving or arriving at the corner base - the segment
    //counted as "blocked" straight away and the robot stopped after a few cm.
    //Only require the clearance that the robot's own cell and the target cell have.
    int r = ROBOT_WIDTH - 1;
    while (r > 0 && !isWalkible(x0, y0, r)) r--;
    while (r > 0 && !isWalkible(x1, y1, r)) r--;

    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int error = dx + dy;
    int x = x0, y = y0;

    while (!(x == x1 && y == y1)) {
        int e2 = 2 * error;
        if (e2 >= dy) { error += dy; x += sx; }
        if (e2 <= dx) { error += dx; y += sy; }
        if (!isWalkible(x, y, r)) return true;   // checks the cell just stepped into
    }
    return false;
}

void findTarget() {
    //get the next target
    if((targetIndex < targetCount-1) && !pathfinished) {
        targetIndex++;
        currentTarget = targetlist[targetIndex];

    } else {  
        pathfinished = 1;
        targetIndex = 0;
    }
}

float findAngle(pos_t* myPose) {
    float dx = currentTarget.x - myPose->x;
    float dy = currentTarget.y - myPose->y;
    return -std::atan2(dx, dy) / D_TO_R;
}

//heading error to the current target, degrees, wrapped to (-180, 180]
static float headingErrorDeg(pos_vector_t* myPose) {
    float error = findAngle(&(myPose->pos)) - (myPose->theta)/D_TO_R;
    while (error > 180.0f)  error -= 360.0f;
    while (error <= -180.0f) error += 360.0f;
    return error;
}

bool rotatePID(pos_vector_t* myPose, float dt) {
    float error = headingErrorDeg(myPose);

    //FIX: only finish once we've stayed inside the tolerance for a few calls.
    //It used to finish the instant it crossed 5 deg, while still spinning fast,
    //so it coasted past and then drove off at the wrong angle.
    if (rotate.tolerance >= fabs(error)) {
        stopMotors();
        settleCount++;
        return settleCount >= ROTATE_SETTLE_COUNT;
    }
    settleCount = 0;

    int motorSpeed = (int)rotate.update(error, dt);
    setDriveSpeed(-motorSpeed, motorSpeed);
    return 0;
}

translateResult translatePID(pos_vector_t* myPose, float dt) {
    float dx = currentTarget.x - myPose->pos.x;
    float dy = currentTarget.y - myPose->pos.y;
    float dist = std::sqrt(dx*dx + dy*dy);

    //check if we are done
    if (translate.tolerance >= dist) return ARRIVED;

    //FIX: use the SIGNED distance along the robot's heading. The old error was
    //sqrt(dx^2+dy^2), which is always positive - after an overshoot the error
    //grew again and the PID drove forward, AWAY from the target, until timeout.
    float theta = myPose->theta;                    //rad, robot faces +y at 0
    float fx = -std::sin(theta), fy = std::cos(theta);
    float along = dx * fx + dy * fy;

    if (along <= 0.0f) {
        //target is now beside/behind us: close enough counts, otherwise re-turn
        return (dist <= 2.0f * translate.tolerance) ? ARRIVED : OVERSHOT;
    }

    float headingErr = headingErrorDeg(myPose);
    if (fabs(headingErr) > RETURN_HEADING_ERR && dist > STEER_MIN_DIST) return NEED_TURN;

    int baseSpeed = (int)translate.update(along, dt);
    if (baseSpeed < MIN_DRIVE_SPEED) baseSpeed = MIN_DRIVE_SPEED;

    //FIX: steer towards the target while driving. Both wheels used to get the same
    //speed, so any drift or turn overshoot was never corrected.
    int turn = (dist > STEER_MIN_DIST) ? (int)steer.update(headingErr, dt) : 0;

    setDriveSpeed(baseSpeed - turn, baseSpeed + turn);
    return DRIVING;
}

void movementTask(float dt) {

    //FIX: only log on change (was ~1.2 kB/s of Bluetooth at 200 Hz)
    if (State != lastLoggedState && BLUETOOTH_SERIAL.availableForWrite() >= 8) {
        BLUETOOTH_SERIAL.print("M, "); BLUETOOTH_SERIAL.println(State);
        lastLoggedState = State;
    }

    pos_vector_t myPOS = getMyPOS();

    if (path_failures >= WAYPOINT_PATIENCE) {
        path_failures = 0;
        stopMotors();
        setIsTurning(0);
        enterState(DONE);
    }

    switch (State) {
        case IDLE:
            {
            rotate.reset();
            translate.reset();
            steer.reset();
            settleCount = 0;

            //if we are too close to target choose a new one
            float dx = currentTarget.x - myPOS.pos.x;
            float dy = currentTarget.y - myPOS.pos.y;
            float dist = std::sqrt(dx*dx + dy*dy);

            enterState((dist <= translate.tolerance) ? DONE : TURNING);
            break;
        }
        case TURNING:
            setIsTurning(1);
            if (millis() - stateStart > TURN_TIME_LIMIT_MS) {
                stopMotors();
                setIsTurning(0);
                path_failures++;
                enterState(IDLE);
                break;
            }
            if (rotatePID(&myPOS, dt)) {
                stopMotors();
                setIsTurning(0);
                translate.reset();
                steer.reset();
                enterState(MOVEING);
            }
            break;
        case MOVEING: {
            if (millis() - stateStart > MOVE_TIME_LIMIT_MS) {
                stopMotors();
                path_failures++;
                enterState(IDLE);           //retry the same waypoint
                break;
            }
            if (pathSegmentBlocked(myPOS.pos, currentTarget)) {
                //FIX: pathBlocked was never set, so a blocked waypoint was just
                //skipped (DONE -> next waypoint). The robot could skip every waypoint,
                //"finish" without moving, and run the arrival action. Now we end the
                //path and flag it so the state machine replans.
                stopMotors();
                pathBlocked = 1;
                pathfinished = 1;
                targetIndex = 0;
                enterState(IDLE);
                return;
            }
            translateResult r = translatePID(&myPOS, dt);
            if (r == ARRIVED) {
                stopMotors();
                path_failures = 0;          //FIX: was never reset after a success
                enterState(DONE);
            } else if (r == OVERSHOT) {
                stopMotors();
                path_failures++;
                enterState(IDLE);           //turn back towards it
            } else if (r == NEED_TURN) {
                stopMotors();
                enterState(IDLE);
            }
            break;
        }
        case DONE:
            findTarget();
            enterState(IDLE);
            break;
    }
}

bool getPathStatus() {
    return pathfinished;
}

bool getPathBlockedStatus() {
    return pathBlocked;
}

void setPath(const pos_t* path, int count) {
    if (count > MAX_WAYPOINT_COUNT) count = MAX_WAYPOINT_COUNT;
    targetCount = count;
    std::copy(path, path + count, targetlist);
    pathfinished = 0;
    pathBlocked = 0;
    currentTarget = targetlist[0];

    //FIX: start every new path from a clean state (index, failures, turn flag)
    targetIndex = 0;
    path_failures = 0;
    setIsTurning(0);
    stopMotors();
    enterState(IDLE);

    for (int i = 0; i < count; i++) {
        int gx, gy;
        worldToGrid(targetlist[i].x, targetlist[i].y, gx, gy);
        BLUETOOTH_SERIAL.print("C, "); BLUETOOTH_SERIAL.print(gx); BLUETOOTH_SERIAL.print(", "); BLUETOOTH_SERIAL.print(gy);
        BLUETOOTH_SERIAL.print(", "); BLUETOOTH_SERIAL.println(52);
    }
}

void stopMovement() {
    stopMotors();
    setIsTurning(0);
    enterState(IDLE);
}
