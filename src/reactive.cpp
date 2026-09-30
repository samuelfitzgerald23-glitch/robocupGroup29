//Reactive controller: bang-bang wall avoidance, weight centering, return to base.
//No map / pathfinding used. Still uses getMyPOS() (optical flow + IMU) for
//turning to a heading and for finding the way home.
//
//Angle convention (same as moving.cpp): theta in degrees, + = CCW = LEFT turn.
//setDriveSpeed(leftWheel, rightWheel), + = forward.

#include <Arduino.h>
#include "reactive.h"
#include "filtering.h"
#include "map.h"
#include "motor.h"
#include "smart_servo.h"
#include "gate.h"
#include "target_finding.h"      //MAX_WEIGHTS_ONBOARD
#include <math.h>
//----------------------------------------------------------------------------
// Sensor mapping for WALL AVOIDANCE - SWAP THESE if avoidance steers the wrong way.
// Assumed: TOF1 looks 45deg RIGHT, TOF2 looks 45deg LEFT (from map.cpp offsets).
//----------------------------------------------------------------------------
#define RIGHT_TOF        TOF1
#define RIGHT_DIFF_TOF   DIFFTOF1
#define LEFT_TOF         TOF2
#define LEFT_DIFF_TOF    DIFFTOF2
#define LEFT_US          ULTRASOUND_LEFT
#define RIGHT_US         ULTRASOUND_RIGHT

//----------------------------------------------------------------------------
// Weight detection sensor mapping (separate so it can be fixed on its own).
// Set WEIGHT_SIDES_SWAPPED to 1 if a weight is detected on the wrong side.
//----------------------------------------------------------------------------
#define WEIGHT_SIDES_SWAPPED   0
#if WEIGHT_SIDES_SWAPPED
  #define W_LEFT_TOF        RIGHT_TOF
  #define W_LEFT_DIFF_TOF   RIGHT_DIFF_TOF
  #define W_RIGHT_TOF       LEFT_TOF
  #define W_RIGHT_DIFF_TOF  LEFT_DIFF_TOF
#else
  #define W_LEFT_TOF        LEFT_TOF
  #define W_LEFT_DIFF_TOF   LEFT_DIFF_TOF
  #define W_RIGHT_TOF       RIGHT_TOF
  #define W_RIGHT_DIFF_TOF  RIGHT_DIFF_TOF
#endif

// Set to -1 if the robot spins the wrong way when told to turn to a heading
// (affects turnTo(): the start turn and the weight centring).
#define TURN_TO_SIGN           1

//----------------------------------------------------------------------------
// Tuning
//----------------------------------------------------------------------------
#define FAR_MM               2000     //value used for "nothing seen" / bad reading

#define CRUISE_SPEED         370      //-450..450
#define STEER_DIFF           150      //wheel speed +/- when nudging away from a wall
#define SPIN_SPEED           350      //in-place spin when blocked

#define FRONT_STOP_MM        300      //ultrasound: start spinning away
#define FRONT_CLEAR_MM       450      //ultrasound: stop spinning (hysteresis)
#define SIDE_WARN_MM         300      //angled TOF: nudge away

#define INITIAL_TURN_DEG     -45.0f    //heading to leave the start area on
#define STARTUP_WAIT_MS      1500     //let filters fill / pose settle

#define WEIGHT_DROP_MM       200      //low TOF this much shorter than top TOF = weight
#define WEIGHT_MAX_RANGE_MM  700      //ignore "weights" further than this
#define WEIGHT_US_MIN_MM     300      //ultrasound must be > this (not a wall)
#define WEIGHT_TOP_TOF_MIN_MM 500     //top TOF must be > this (weight is low)
#define WEIGHT_CONFIRM_MS    30       //must be seen this long (reduced for faster response)
#define WEIGHT_TURN_LEFT_DEG   45.0f  //LEFT diff TOF sees weight  -> turn this far ANTICLOCKWISE
#define WEIGHT_TURN_RIGHT_DEG  45.0f  //RIGHT diff TOF sees weight -> turn this far CLOCKWISE
#define WEIGHT_COOLDOWN_MS   500     //ignore weights after a missed attempt
#define PICKUP_COOLDOWN_MS   500      //short ignore-weights time after a pickup starts
#define CHASE_WHILE_SORTING  1        //1 = keep hunting weights while the servo is busy
                                      //0 = don't turn toward new weights until servo is free
                                      //    (still keeps moving / avoiding walls either way

#define APPROACH_SPEED       360
#define APPROACH_MS          4000     //drive forward for pickup
#define APPROACH_WALL_MM     120      //abort approach if this close ahead

#define TURN_TOL_DEG         8.0f     //increased tolerance to prevent overshoot
#define TURN_KP              3.0f     //wheel speed per degree of error
#define TURN_MIN_SPEED       120      //reduced to avoid overshoot at small errors
#define TURN_MAX_SPEED       360
#define TURN_TIMEOUT_MS      2000

#define HOME_HEADING_TOL_DEG 8.0f    //drive straight if within this of home bearing
#define HOME_IGNORE_MM       300      //near base: stop avoiding walls so we can dock


#define STUCK_TIMEOUT_MS       8000UL
#define REVERSE_RECOVERY_MS    500UL
#define STUCK_MOVE_MM          20.0f
#define STUCK_TURN_DEG         5.0f
#define RECOVERY_SPEED         250

static uint32_t lastProgressTime = 0;
static uint32_t recoveryStartTime = 0;
static float progressX = 0;
static float progressY = 0;
static float progressHeading = 0;
static bool recovering = false;
static bool progressInitialized = false;
static bool homeTurnStarted = false;

//----------------------------------------------------------------------------
enum ReactState {
    R_STARTUP, R_LEAVE_START, R_EXPLORE, R_CENTER_WEIGHT,
    R_APPROACH_WEIGHT, R_GO_HOME, R_DROP
};

static ReactState state = R_STARTUP;
static uint32_t   stateStart = 0;
static float      turnTarget = 0;
static int        weightSide = 0;          //+1 = left, -1 = right
static uint32_t   weightCooldownUntil = 0;

static void enter(ReactState s) {
    state = s;
    stateStart = millis();
}

//----------------------------------------------------------------------------
// Helpers
//----------------------------------------------------------------------------
static float wrap180(float a) {
    while (a > 180.0f)   a -= 360.0f;
    while (a <= -180.0f) a += 360.0f;
    return a;
}

static int rd(BufID id) {            //filtered reading, bad/zero -> "far"
    int v = getOutput(id);
    return (v <= 0 || v > FAR_MM) ? FAR_MM : v;
}


static void driveWithRecovery(int left, int right) {
    uint32_t now = millis();
    pos_vector_t me = getMyPOS();
    float heading = me.theta * RAD_TO_DEG;

    // While recovering, override all normal motor commands.
    if (recovering) {
        if (now - recoveryStartTime < REVERSE_RECOVERY_MS) {
            setDriveSpeed(-RECOVERY_SPEED, -RECOVERY_SPEED);
            return;
        }

        recovering = false;
        lastProgressTime = now;
        progressX = me.pos.x;
        progressY = me.pos.y;
        progressHeading = heading;
    }

    // Intentional stop: don't count it as being stuck.
    if (left == 0 && right == 0) {
        lastProgressTime = now;
        progressX = me.pos.x;
        progressY = me.pos.y;
        progressHeading = heading;
        setDriveSpeed(0, 0);
        return;
    }

    if (!progressInitialized) {
        progressX = me.pos.x;
        progressY = me.pos.y;
        progressHeading = heading;
        lastProgressTime = now;
        progressInitialized = true;
    }

    float dx = me.pos.x - progressX;
    float dy = me.pos.y - progressY;
    float moved = sqrtf(dx * dx + dy * dy);
    float turned = fabsf(wrap180(heading - progressHeading));

    // Translation OR rotation counts as progress.
    if (moved >= STUCK_MOVE_MM || turned >= STUCK_TURN_DEG) {
        lastProgressTime = now;
        progressX = me.pos.x;
        progressY = me.pos.y;
        progressHeading = heading;
    }

    // No progress for too long: reverse briefly.
    if (now - lastProgressTime >= STUCK_TIMEOUT_MS) {
        recovering = true;
        recoveryStartTime = now;
        lastProgressTime = now;

        Serial.println("Stuck: reversing for recovery");
        setDriveSpeed(-RECOVERY_SPEED, -RECOVERY_SPEED);
        return;
    }

    setDriveSpeed(left, right);
}

static void spin(int dir, int speed) {       //dir +1 = left (CCW), -1 = right
    driveWithRecovery(-dir * speed, dir * speed);
}


//proportional in-place turn to an absolute heading. true when there.
//improved control to prevent overshoot by using lower speeds at small errors
static bool turnTo(float targetDeg) {
    float err = wrap180(targetDeg - getMyPOS().theta * RAD_TO_DEG);   //theta is radians
    if (fabsf(err) <= TURN_TOL_DEG) {
        setDriveSpeed(0, 0);
        return true;
    }
    //reduce speed for fine control: 30+ deg = full speed, 10 deg = min speed, smooth scaling
    float absErr = fabsf(err);
    int s;
    if (absErr > 30.0f) {
        s = TURN_MAX_SPEED;
    } else if (absErr > 15.0f) {
        s = constrain((int)(TURN_MIN_SPEED + (absErr - 15.0f) * 6), TURN_MIN_SPEED, TURN_MAX_SPEED);
    } else {
        s = TURN_MIN_SPEED;
    }
    spin((err > 0 ? 1 : -1) * TURN_TO_SIGN, s);
    return false;
}

//----------------------------------------------------------------------------
// Bang-bang wall avoidance. Returns true if it took over the motors.
//   1. ultrasounds (forward) too close -> spin toward the roomier side (latched)
//   2. angled TOF on one side too close -> nudge away from it
//----------------------------------------------------------------------------
static bool avoidWalls(int cruise) {
    static int spinDir = 0;

    int usL = rd(LEFT_US),  usR = rd(RIGHT_US);
    int tL  = rd(LEFT_TOF), tR  = rd(RIGHT_TOF);
    int front = min(usL, usR);

    if (spinDir != 0) {                       //already spinning: hold until clear
        if (front >= FRONT_CLEAR_MM) spinDir = 0;
        else { spin(spinDir, SPIN_SPEED); return true; }
    }

    if (front < FRONT_STOP_MM) {
        spinDir = ((usL + tL) > (usR + tR)) ? 1 : -1;   //toward the side with more room
        spin(spinDir, SPIN_SPEED);
        return true;
    }

    bool rightClose = (tR < SIDE_WARN_MM);
    bool leftClose  = (tL < SIDE_WARN_MM);

    if (rightClose && !leftClose) {           //wall on right -> steer left
        driveWithRecovery(cruise - STEER_DIFF, cruise + STEER_DIFF);
        return true;
    }
    if (leftClose && !rightClose) {           //wall on left -> steer right
        driveWithRecovery(cruise + STEER_DIFF, cruise - STEER_DIFF);
        return true;
    }
    return false;
}

//----------------------------------------------------------------------------
// Weight detection: low (diff) TOF sees something much closer than the top TOF,
// AND ultrasounds read far (no wall), AND upper TOF reads far (object at foot level).
// Returns +1 (weight on left), -1 (weight on right), 0 (none). Debounced.
//----------------------------------------------------------------------------
static bool seesWeight(BufID top, BufID low, BufID us, int& lowDist) {
    int t = rd(top);
    lowDist = rd(low);
    int ultrasound = rd(us);
    
    // Weight signature: diff TOF low, top TOF high, ultrasound high (not a wall)
    return (lowDist < t - WEIGHT_DROP_MM) &&       // significant height difference
           (lowDist < WEIGHT_MAX_RANGE_MM) &&      // within range
           (ultrasound >= WEIGHT_US_MIN_MM) &&     // ultrasound says no wall
           (t >= WEIGHT_TOP_TOF_MIN_MM);           // top TOF sees far (weight is low)
}

static int checkWeights() {
    static uint32_t since = 0;
    static int lastSide = 0;

    int dL, dR;
    bool l = seesWeight(W_LEFT_TOF,  W_LEFT_DIFF_TOF,  LEFT_US,  dL);
    bool r = seesWeight(W_RIGHT_TOF, W_RIGHT_DIFF_TOF, RIGHT_US, dR);

    int side = 0;
    if (l && r)      side = (dL <= dR) ? 1 : -1;   //both: take the closer one
    else if (l)      side = 1;
    else if (r)      side = -1;

    if (side == 0 || side != lastSide) {
        lastSide = side;
        since = millis();
        return 0;
    }
    return (millis() - since >= WEIGHT_CONFIRM_MS) ? side : 0;
}

//----------------------------------------------------------------------------
// Main
//----------------------------------------------------------------------------
void reactiveTask() {
    pos_vector_t me = getMyPOS();
    float th = me.theta * RAD_TO_DEG;
    uint32_t elapsed = millis() - stateStart;

    //full? go home (from any driving state)
    if ((state == R_EXPLORE || state == R_LEAVE_START) &&
        get_num_weights() >= MAX_WEIGHTS_ONBOARD) {
        setDriveSpeed(0, 0);
        enter(R_GO_HOME);
    }

    switch (state) {

    case R_STARTUP:
        setDriveSpeed(0, 0);
        if (millis() >= STARTUP_WAIT_MS) {
            //start LEFT -> head out to the right (-), start RIGHT -> head out left (+)
            turnTarget = START_ON_LEFT ? -INITIAL_TURN_DEG : INITIAL_TURN_DEG;
            enter(R_LEAVE_START);
        }
        break;

    case R_LEAVE_START:
        if (turnTo(turnTarget) || elapsed > TURN_TIMEOUT_MS) {
            setDriveSpeed(0, 0);
            enter(R_EXPLORE);
        }
        break;

    case R_EXPLORE: {
        if (millis() > weightCooldownUntil &&
            (CHASE_WHILE_SORTING || !isServoSequenceActive())) {
            int side = checkWeights();
            if (side != 0) {
                weightSide = side;
                float turn = (side > 0) ? WEIGHT_TURN_LEFT_DEG        // + = anticlockwise
                                        : -WEIGHT_TURN_RIGHT_DEG;     // - = clockwise
                turnTarget = wrap180(th + turn);
                Serial.print(side > 0 ? "Weight on LEFT, " : "Weight on RIGHT, ");
                Serial.print("theta "); Serial.print(th);
                Serial.print(" -> target "); Serial.println(turnTarget);
                setDriveSpeed(0, 0);
                enter(R_CENTER_WEIGHT);
                break;
            }
        }
        if (!avoidWalls(CRUISE_SPEED)) driveWithRecovery(CRUISE_SPEED, CRUISE_SPEED);
        break;
    }

    case R_CENTER_WEIGHT:
        if (turnTo(turnTarget) || elapsed > TURN_TIMEOUT_MS) {
            setDriveSpeed(0, 0);
            enter(R_APPROACH_WEIGHT);
        }
        break;

    case R_APPROACH_WEIGHT:
        if (isServoSequenceActive()) {                 //picked up: go back to exploring
            weightCooldownUntil = millis() + PICKUP_COOLDOWN_MS;
            enter(R_EXPLORE);
        } else if (elapsed > APPROACH_MS ||
            min(rd(LEFT_US), rd(RIGHT_US)) < APPROACH_WALL_MM) {
            setDriveSpeed(0, 0);                       //missed it / wall ahead
            weightCooldownUntil = millis() + WEIGHT_COOLDOWN_MS;
            enter(R_EXPLORE);
        } else {
            // Just drive straight forward, no heading correction needed
            driveWithRecovery(APPROACH_SPEED, APPROACH_SPEED);
        }
        break;

    case R_GO_HOME: {
        int usL = rd(LEFT_US);
        int usR = rd(RIGHT_US);

        // At base: stop and turn 180 before starting the drop
        if (usL <= 300 && usR <= 300) {
            setDriveSpeed(0, 0);

            if (!homeTurnStarted) {
                turnTarget = wrap180(th + 180.0f);
                homeTurnStarted = true;
            }

            if (turnTo(turnTarget)) {
                setDriveSpeed(0, 0);
                homeTurnStarted = false;
                gateStartDrop();
                enter(R_DROP);
            }
            break;
        }

        // If the base turn has started, finish it before navigating again
        if (homeTurnStarted) {
            turnTo(turnTarget);
            break;
        }

        // Calculate direction toward home (world origin)
        float dx = 0.0f - me.pos.x;
        float dy = 0.0f - me.pos.y;

        float bearing = -atan2f(dx, dy) * RAD_TO_DEG;
        float err = wrap180(bearing - th);

        // Turn toward home, then drive straight
        if (fabsf(err) > HOME_HEADING_TOL_DEG) {
            spin(err > 0 ? 1 : -1, SPIN_SPEED);
        } else {
            driveWithRecovery(CRUISE_SPEED, CRUISE_SPEED);
        }

        break;
    }

    case R_DROP:

        if (gateDrop()) {
            reset_num_weights();
            homeTurnStarted = false;
            turnTarget = START_ON_LEFT ? -INITIAL_TURN_DEG : INITIAL_TURN_DEG;
            enter(R_LEAVE_START);
        } else if (elapsed > 25000UL) {                    //jam safety net
            setDriveSpeed(0, 0);
            enter(R_EXPLORE);
        }
        break;
    }
}

