//Reactive controller: wall following + avoidance, weight centring, return to base.
//No map / pathfinding used. Still uses getMyPOS() (optical flow + IMU) for
//turning to a heading, aiming at weights and finding the way home.
//
//Angle convention (same as moving.cpp): theta in degrees, + = CCW = LEFT turn.
//drive(leftWheel, rightWheel), + = forward.
//Robot frame: +x = right, +y = forward (same as map.cpp sensor offsets).
//
//Motor control is layered, highest priority first:
//  1. safety   - back up / spin away when something is dangerously close
//  2. follow   - PD control holding a steady distance from the nearest side wall
//  3. cruise   - straight ahead, slowing down as the front gets close

#include <Arduino.h>
#include "reactive.h"
#include "filtering.h"
#include "map.h"
#include "motor.h"
#include "smart_servo.h"
#include "gate.h"
#include "target_finding.h"      //MAX_WEIGHTS_ONBOARD

//----------------------------------------------------------------------------
// Sensor mapping - SWAP THESE if the robot steers the wrong way near walls.
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
// Sensor geometry (robot frame, mm) - must match the TOF offsets in map.cpp
//----------------------------------------------------------------------------
#define TOF_X_OFFSET         105.0f   //side TOFs are this far left/right of centre
#define TOF_Y_OFFSET         170.0f   //...and this far forward
#define TOF_ANGLE_DEG        45.0f    //...angled this far out from straight ahead
#define WEIGHT_RADIUS_MM     25.0f    //aim at the weight's centre, not its surface

//----------------------------------------------------------------------------
// Tuning - starting points, tune on the field (the "R," telemetry line helps)
//----------------------------------------------------------------------------
#define FAR_MM               2000     //value used for "nothing seen" / bad reading
#define MIN_VALID_MM         30       //low TOF readings below this are errors

#define CRUISE_SPEED         300      //-450..450
#define MIN_SPEED            120      //slowest forward speed when something is ahead
#define SPIN_SPEED           250      //in-place spin when blocked
#define REVERSE_SPEED        200
#define REVERSE_MS           350      //back up this long when something is very close ahead
#define ESCAPE_MAX_MS        3000     //spinning this long without clearing -> back up

//stall detection: driving/turning but the optical flow + IMU say we're not moving
//(pushing a wall/obstacle corner that no sensor can see, e.g. with a front corner)
#define STUCK_MS             800      //no progress for this long = stuck
#define STUCK_MOVE_MM        30       //must move at least this far in STUCK_MS
#define STUCK_TURN_DEG       8.0f     //...or turn at least this much (a spin can jam too)
#define STUCK_MIN_CMD        100      //only check when commanding at least this speed
#define RECOVER_MOVE_MS      500      //then back up this long...
#define RECOVER_SPIN_MS      600      //...and spin this long toward the roomier side
#define RECOVER_STREAK_MS    4000     //stuck again within this time = that didn't work, so
                                      //alternate: drive FORWARD and spin the other way
                                      //(the thing we hit may be behind us - no rear sensors)

//front (ultrasounds, straight ahead)
#define FRONT_CRITICAL_MM    110      //back up
#define FRONT_STOP_MM        230      //spin away
#define FRONT_CLEAR_MM       400      //stop spinning (hysteresis)
#define FRONT_SLOW_MM        650      //start slowing down below this

//sides (45 deg TOFs - range measured ALONG the angled beam)
#define SIDE_EMERGENCY_MM    170      //about to scrape: spin away
#define SIDE_CLEAR_MM        280      //stop spinning
#define CORNER_MM            320      //BOTH side TOFs below this = wall/corner ahead
                                      //(catches walls the ultrasounds miss at an angle)

//wall following (holds the side TOF range at FOLLOW_SET_MM)
#define FOLLOW_ACQUIRE_MM    600      //start following a wall seen closer than this
#define FOLLOW_LOSE_MM       850      //stop following once it's further than this
#define FOLLOW_SET_MM        330      //target range along the 45 deg beam
                                      //(~ 330*0.7 = 230 mm sideways from the sensor)
#define FOLLOW_KP            0.8f     //wheel speed difference per mm of error
#define FOLLOW_KD            0.25f    //... per mm/s the range is changing (damping)
#define FOLLOW_MAX_ERR       200.0f   //cap so a far wall doesn't cause a hard swerve
#define FOLLOW_STEER_MAX     170

#define INITIAL_TURN_DEG     45.0f    //heading to leave the start area on
#define STARTUP_WAIT_MS      1500     //let filters fill / pose settle

//weights: low (diff) TOF sees something much closer than the top TOF above it
#define WEIGHT_DROP_MM       200      //low TOF this much shorter than top TOF = weight
#define WEIGHT_MAX_RANGE_MM  700      //ignore "weights" further than this
#define WEIGHT_CONFIRM_SAMPLES 6      //must be seen in this many new readings in a row...
#define WEIGHT_CONFIRM_MS    150      //...spanning at least this long...
#define WEIGHT_CONSISTENT_MM 80       //...at a consistent distance
#define WEIGHT_COOLDOWN_MS   2000     //ignore weights after a missed attempt
#define PICKUP_COOLDOWN_MS   500      //short ignore-weights time after a pickup starts
#define CHASE_WHILE_SORTING  1        //1 = keep hunting weights while the servo is busy
                                      //0 = don't turn toward new weights until servo is free
                                      //    (still keeps moving / avoiding walls either way)

#define APPROACH_SPEED       220
#define APPROACH_KP          4.0f     //steering per degree while driving at the weight
#define APPROACH_PAST_MM     150      //keep going until the weight is this far behind centre
#define APPROACH_MAX_MS      4000
#define APPROACH_WALL_MM     120      //abort approach if this close ahead

#define TURN_TOL_DEG         4.0f
#define TURN_KP              4.0f     //wheel speed per degree of error
#define TURN_MIN_SPEED       140
#define TURN_MAX_SPEED       300
#define TURN_TIMEOUT_MS      3000

#define HOME_SPIN_DEG        60.0f    //spin in place if home is further off than this
#define HOME_KP              4.0f     //steering per degree when heading home
#define HOME_WALL_PUSH       80       //extra steering away from a side wall on the way home
#define HOME_SIDE_WARN_MM    300
#define HOME_IGNORE_MM       300      //near base: stop avoiding walls so we can dock
#define HOME_SLOW_MM         500      //slow down this close to base
#define HOME_SLOW_SPEED      180
#define HOME_FOLLOW_MS       2500     //blocked on the way home: follow the obstacle's edge
                                      //this long before trying to head home again

#define TELEMETRY_MS         100      //"R," line over Bluetooth this often

//----------------------------------------------------------------------------
enum ReactState {
    R_STARTUP, R_LEAVE_START, R_EXPLORE, R_CENTER_WEIGHT,
    R_APPROACH_WEIGHT, R_GO_HOME, R_DROP
};

static ReactState state = R_STARTUP;
static uint32_t   stateStart = 0;
static float      turnTarget = 0;
static pos_t      weightTarget = {0, 0};     //world position of the weight we're chasing
static uint32_t   weightCooldownUntil = 0;
static uint32_t   homeFollowUntil = 0;       //see R_GO_HOME

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

static int cmdL = 0, cmdR = 0;              //last commanded wheel speeds

static void drive(int left, int right) {
    cmdL = left;
    cmdR = right;
    setDriveSpeed(left, right);
}

static void spin(int dir, int speed) {       //dir +1 = left (CCW), -1 = right
    drive(-dir * speed, dir * speed);
}

//heading (deg) from the robot to a world point
static float bearingTo(pos_t p, const pos_vector_t& me) {
    float dx = p.x - me.pos.x;
    float dy = p.y - me.pos.y;
    return -atan2f(dx, dy) * RAD_TO_DEG;     //same convention as moving.cpp
}

//proportional in-place turn to an absolute heading. true when there.
static bool turnTo(float targetDeg) {
    float err = wrap180(targetDeg - getMyPOS().theta * RAD_TO_DEG);   //theta is radians
    if (fabsf(err) <= TURN_TOL_DEG) {
        drive(0, 0);
        return true;
    }
    int s = constrain((int)(TURN_KP * fabsf(err)), TURN_MIN_SPEED, TURN_MAX_SPEED);
    spin((err > 0 ? 1 : -1) * TURN_TO_SIGN, s);
    return false;
}

//----------------------------------------------------------------------------
// Sensor snapshot. Readings only change when filterTask runs, so everything
// that needs "new data" (rates, weight confirmation) works off this.
//----------------------------------------------------------------------------
struct Readings { int usL, usR, tL, tR; };
static Readings rdg = {FAR_MM, FAR_MM, FAR_MM, FAR_MM};
static float    rateL = 0, rateR = 0;        //side TOF range change, mm/s (- = closing)
static bool     freshData = false;
static uint32_t lastFilterCount = 0;
static uint32_t lastSampleMs = 0;

static void updateReadings() {
    uint32_t c = getFilterUpdateCount();
    freshData = (c != lastFilterCount);
    if (!freshData) return;
    lastFilterCount = c;

    uint32_t now = millis();
    float dt = (now - lastSampleMs) / 1000.0f;
    lastSampleMs = now;

    Readings n = {rd(LEFT_US), rd(RIGHT_US), rd(LEFT_TOF), rd(RIGHT_TOF)};
    if (dt > 0.001f && dt < 0.5f) {
        float rl = constrain((n.tL - rdg.tL) / dt, -1500.0f, 1500.0f);
        float rr = constrain((n.tR - rdg.tR) / dt, -1500.0f, 1500.0f);
        rateL = 0.6f * rateL + 0.4f * rl;     //smoothed
        rateR = 0.6f * rateR + 0.4f * rr;
    }
    rdg = n;
}

static int frontDist() { return min(rdg.usL, rdg.usR); }
static int sideMin()   { return min(rdg.tL, rdg.tR); }
static bool cornerAhead() { return rdg.tL < CORNER_MM && rdg.tR < CORNER_MM; }
static int roomierSide()  { return (rdg.tL > rdg.tR) ? 1 : -1; }   //+1 = left

//forward speed, slowing down as the front gets close
static int speedForFront(int front, int cruise) {
    if (front >= FRONT_SLOW_MM) return cruise;
    float k = (float)(front - FRONT_STOP_MM) / (FRONT_SLOW_MM - FRONT_STOP_MM);
    k = constrain(k, 0.0f, 1.0f);
    return MIN_SPEED + (int)(k * (cruise - MIN_SPEED));
}

//----------------------------------------------------------------------------
// Layer 1: safety. Returns true if it took over the motors.
//----------------------------------------------------------------------------
static int      escapeDir = 0;        //latched spin direction, 0 = not escaping
static uint32_t escapeStart = 0;
static uint32_t reverseUntil = 0;
static uint32_t recoverMoveUntil = 0;  //stall recovery: straight move (back or forward)...
static int      recoverMoveSpeed = 0;
static uint32_t recoverSpinUntil = 0;  //...then a forced spin (sensors may see nothing)
static int      recoverDir = 0;
static uint32_t lastStuckMs = 0;
static int      stuckStreak = 0;

static bool safetyActive() {
    uint32_t now = millis();
    return escapeDir != 0 || now < reverseUntil || now < recoverMoveUntil || now < recoverSpinUntil;
}

//true (once) when we've been driving or turning without actually moving/turning.
//Starts the recovery.
static pos_t    stuckRef = {0, 0};
static float    stuckRefTheta = 0;
static uint32_t stuckRefMs = 0;

static bool checkStuck(const pos_vector_t& me) {
    uint32_t now = millis();
    bool commanding = max(abs(cmdL), abs(cmdR)) >= STUCK_MIN_CMD;
    float moved  = hypotf(me.pos.x - stuckRef.x, me.pos.y - stuckRef.y);
    float turned = fabsf(wrap180((me.theta - stuckRefTheta) * RAD_TO_DEG));

    if (!commanding || moved > STUCK_MOVE_MM || turned > STUCK_TURN_DEG) {
        stuckRef = me.pos;
        stuckRefTheta = me.theta;
        stuckRefMs = now;
        return false;
    }
    if (now - stuckRefMs < STUCK_MS) return false;

    //stuck: back up, then spin toward the side with more room. If that already
    //failed recently, try the opposite (forward + other way) on alternate attempts.
    stuckStreak = (now - lastStuckMs < RECOVER_STREAK_MS) ? stuckStreak + 1 : 0;
    lastStuckMs = now;
    bool alternate = (stuckStreak % 2) == 1;
    bool forward   = alternate && frontDist() > FRONT_STOP_MM;

    recoverMoveSpeed = forward ? REVERSE_SPEED : -REVERSE_SPEED;
    recoverMoveUntil = now + RECOVER_MOVE_MS;
    recoverSpinUntil = recoverMoveUntil + RECOVER_SPIN_MS;
    recoverDir       = alternate ? -roomierSide() : roomierSide();
    escapeDir = 0;                                    //recovery replaces any escape spin
    stuckRef = me.pos;
    stuckRefTheta = me.theta;
    stuckRefMs = now;
    return true;
}

static void startEscape(int dir) {
    if (escapeDir == 0) escapeStart = millis();
    escapeDir = dir;
}

static bool safety() {
    uint32_t now = millis();
    int front = frontDist();

    if (now < reverseUntil) {                         //backing up
        drive(-REVERSE_SPEED, -REVERSE_SPEED);
        return true;
    }
    if (now < recoverMoveUntil) {                     //stall recovery: straight move
        drive(recoverMoveSpeed, recoverMoveSpeed);
        return true;
    }
    if (now < recoverSpinUntil) {                     //turning away after a stall
        spin(recoverDir, SPIN_SPEED);
        return true;
    }

    if (escapeDir != 0) {                             //spinning: hold until clear
        bool clear = front >= FRONT_CLEAR_MM && sideMin() >= SIDE_CLEAR_MM && !cornerAhead();
        if (clear) {
            escapeDir = 0;
        } else if (now - escapeStart > ESCAPE_MAX_MS) {
            reverseUntil = now + REVERSE_MS;          //stuck spinning: back up, try again
            escapeStart = now;
            drive(-REVERSE_SPEED, -REVERSE_SPEED);
            return true;
        } else {
            spin(escapeDir, SPIN_SPEED);
            return true;
        }
    }

    if (front < FRONT_CRITICAL_MM) {                  //far too close: back up first
        reverseUntil = now + REVERSE_MS;
        startEscape(roomierSide());
        drive(-REVERSE_SPEED, -REVERSE_SPEED);
        return true;
    }

    if (front < FRONT_STOP_MM || cornerAhead() || sideMin() < SIDE_EMERGENCY_MM) {
        startEscape(roomierSide());                   //spin toward the side with more room
        spin(escapeDir, SPIN_SPEED);
        return true;
    }
    return false;
}

//----------------------------------------------------------------------------
// Layers 2 + 3: follow the nearest side wall, otherwise cruise straight.
//----------------------------------------------------------------------------
static int followSide = 0;            //+1 = following a wall on the left, -1 = right

static void driveExplore() {
    int v = speedForFront(frontDist(), CRUISE_SPEED);

    //pick / keep a wall (hysteresis so it doesn't flick between walls)
    if (followSide ==  1 && rdg.tL > FOLLOW_LOSE_MM) followSide = 0;
    if (followSide == -1 && rdg.tR > FOLLOW_LOSE_MM) followSide = 0;
    if (followSide == 0 && (rdg.tL < FOLLOW_ACQUIRE_MM || rdg.tR < FOLLOW_ACQUIRE_MM)) {
        followSide = (rdg.tL <= rdg.tR) ? 1 : -1;
    }

    if (followSide == 0) {
        drive(v, v);
        return;
    }

    //45 deg range works as a look-ahead: heading into the wall shortens it early
    float range = (followSide > 0) ? rdg.tL : rdg.tR;
    float rate  = (followSide > 0) ? rateL  : rateR;
    float err   = constrain(range - FOLLOW_SET_MM, -FOLLOW_MAX_ERR, FOLLOW_MAX_ERR);

    //+ = turn TOWARD the wall (too far), - = turn away (too close / closing fast)
    float steer = FOLLOW_KP * err + FOLLOW_KD * rate;
    steer = constrain(steer, (float)-FOLLOW_STEER_MAX, (float)FOLLOW_STEER_MAX);

    int s = (int)steer * followSide;                  //convert to + = turn left
    drive(v - s, v + s);
}

//----------------------------------------------------------------------------
// Weight detection. Only evaluated on NEW filtered readings. Returns +1 (left),
// -1 (right) or 0, and the low TOF distance once a weight is confirmed.
//----------------------------------------------------------------------------
static int      wSide = 0, wCount = 0;
static uint32_t wStart = 0;
static float    wDist = 0;

static bool lowSeesWeight(BufID top, BufID low, int& lowDist) {
    int t = rd(top);
    lowDist = rd(low);
    return lowDist >= MIN_VALID_MM && lowDist <= WEIGHT_MAX_RANGE_MM &&
           lowDist < t - WEIGHT_DROP_MM;
}

static void resetWeightDetection() {
    wSide = 0;
    wCount = 0;
}

static int checkWeights(float& dist) {
    int dL, dR;
    bool l = lowSeesWeight(W_LEFT_TOF,  W_LEFT_DIFF_TOF,  dL);
    bool r = lowSeesWeight(W_RIGHT_TOF, W_RIGHT_DIFF_TOF, dR);

    int side = 0;
    if (l && r)      side = (dL <= dR) ? 1 : -1;   //both: take the closer one
    else if (l)      side = 1;
    else if (r)      side = -1;

    if (side == 0) { resetWeightDetection(); return 0; }

    float d = (side > 0) ? dL : dR;
    if (side != wSide || fabsf(d - wDist) > WEIGHT_CONSISTENT_MM) {
        wSide = side;                               //new / jumped: start confirming again
        wCount = 1;
        wStart = millis();
        wDist = d;
        return 0;
    }
    wCount++;
    wDist = 0.5f * (wDist + d);

    if (wCount >= WEIGHT_CONFIRM_SAMPLES && millis() - wStart >= WEIGHT_CONFIRM_MS) {
        dist = wDist;
        resetWeightDetection();
        return side;
    }
    return 0;
}

//world position of a weight seen by the low TOF on `side` at `lowDist`
static pos_t weightWorldPos(int side, float lowDist, const pos_vector_t& me) {
    float a  = side * TOF_ANGLE_DEG * DEG_TO_RAD;      //+ = angled left
    float sx = -side * TOF_X_OFFSET;                   //left sensor is at -x
    float r  = lowDist + WEIGHT_RADIUS_MM;
    float px = sx - sinf(a) * r;                       //robot frame
    float py = TOF_Y_OFFSET + cosf(a) * r;

    float c = cosf(me.theta), s = sinf(me.theta);      //robot -> world
    return { me.pos.x + px * c - py * s, me.pos.y + px * s + py * c };
}

//----------------------------------------------------------------------------
// Telemetry: R, state, usL, usR, tofL, tofR, lowL, lowR, followSide, escapeDir
//----------------------------------------------------------------------------
static void telemetry() {
    static uint32_t last = 0;
    if (millis() - last < TELEMETRY_MS) return;
    last = millis();
    if (BLUETOOTH_SERIAL.availableForWrite() < 48) return;
    BLUETOOTH_SERIAL.print("R, ");  BLUETOOTH_SERIAL.print((int)state);
    BLUETOOTH_SERIAL.print(", ");   BLUETOOTH_SERIAL.print(rdg.usL);
    BLUETOOTH_SERIAL.print(", ");   BLUETOOTH_SERIAL.print(rdg.usR);
    BLUETOOTH_SERIAL.print(", ");   BLUETOOTH_SERIAL.print(rdg.tL);
    BLUETOOTH_SERIAL.print(", ");   BLUETOOTH_SERIAL.print(rdg.tR);
    BLUETOOTH_SERIAL.print(", ");   BLUETOOTH_SERIAL.print(rd(LEFT_DIFF_TOF));
    BLUETOOTH_SERIAL.print(", ");   BLUETOOTH_SERIAL.print(rd(RIGHT_DIFF_TOF));
    BLUETOOTH_SERIAL.print(", ");   BLUETOOTH_SERIAL.print(followSide);
    BLUETOOTH_SERIAL.print(", ");   BLUETOOTH_SERIAL.println(escapeDir);
}

//----------------------------------------------------------------------------
// Main
//----------------------------------------------------------------------------
void reactiveTask() {
    updateReadings();
    telemetry();

    pos_vector_t me = getMyPOS();
    float th = me.theta * RAD_TO_DEG;
    uint32_t elapsed = millis() - stateStart;

    bool stuck = checkStuck(me);
    if (stuck) BLUETOOTH_SERIAL.println("R, STUCK - backing up");

    //full? go home (from any driving state)
    if ((state == R_EXPLORE || state == R_LEAVE_START) &&
        get_num_weights() >= MAX_WEIGHTS_ONBOARD) {
        drive(0, 0);
        enter(R_GO_HOME);
    }

    switch (state) {

    case R_STARTUP:
        drive(0, 0);
        if (millis() >= STARTUP_WAIT_MS) {
            //start LEFT -> head out to the right (-), start RIGHT -> head out left (+)
            turnTarget = START_ON_LEFT ? -INITIAL_TURN_DEG : INITIAL_TURN_DEG;
            enter(R_LEAVE_START);
        }
        break;

    case R_LEAVE_START:
        if (turnTo(turnTarget) || elapsed > TURN_TIMEOUT_MS) {
            drive(0, 0);
            enter(R_EXPLORE);
        }
        break;

    case R_EXPLORE: {
        if (freshData && !safetyActive() && millis() > weightCooldownUntil &&
            (CHASE_WHILE_SORTING || !isServoSequenceActive())) {
            float d;
            int side = checkWeights(d);
            if (side != 0) {
                weightTarget = weightWorldPos(side, d, me);
                Serial.print(side > 0 ? "Weight on LEFT at " : "Weight on RIGHT at ");
                Serial.print(d); Serial.print(" mm -> world (");
                Serial.print(weightTarget.x); Serial.print(", ");
                Serial.print(weightTarget.y); Serial.println(")");
                drive(0, 0);
                enter(R_CENTER_WEIGHT);
                break;
            }
        }
        if (safetyActive()) resetWeightDetection();    //readings smear while spinning
        if (!safety()) driveExplore();
        break;
    }

    case R_CENTER_WEIGHT:
        //aim at the weight's calculated position (re-computed as we turn),
        //instead of a fixed 30 deg - a 45 deg sensor sees it ~40 deg off centre
        if (turnTo(bearingTo(weightTarget, me)) || elapsed > TURN_TIMEOUT_MS) {
            drive(0, 0);
            enter(R_APPROACH_WEIGHT);
        }
        break;

    case R_APPROACH_WEIGHT: {
        float dx = weightTarget.x - me.pos.x;
        float dy = weightTarget.y - me.pos.y;
        float dist = sqrtf(dx * dx + dy * dy);
        float along = dx * -sinf(me.theta) + dy * cosf(me.theta);   //+ = still ahead

        if (isServoSequenceActive()) {                 //picked up: keep moving, don't stop
            weightCooldownUntil = millis() + PICKUP_COOLDOWN_MS;
            enter(R_EXPLORE);
        } else if (along < -APPROACH_PAST_MM || elapsed > APPROACH_MAX_MS) {
            //drove through where the weight was (it's in the intake, or we missed it)
            weightCooldownUntil = millis() + WEIGHT_COOLDOWN_MS;
            enter(R_EXPLORE);
        } else if (stuck || frontDist() < APPROACH_WALL_MM || sideMin() < SIDE_EMERGENCY_MM) {
            drive(0, 0);                       //wall in the way - give up
            weightCooldownUntil = millis() + WEIGHT_COOLDOWN_MS;
            enter(R_EXPLORE);
        } else {
            //steer at the weight while there's room to (direction is noisy up close)
            float err = wrap180(bearingTo(weightTarget, me) - th);
            int steer = (dist > 150.0f) ? constrain((int)(APPROACH_KP * err), -120, 120) : 0;
            drive(APPROACH_SPEED - steer, APPROACH_SPEED + steer);
        }
        break;
    }

    case R_GO_HOME: {
        float homeDist = hypotf(me.pos.x, me.pos.y);
        //stuck right next to base = pressed into the corner: close enough to drop
        if (isAtBase() || (stuck && homeDist < HOME_IGNORE_MM)) {
            drive(0, 0);
            gateStartDrop();
            enter(R_DROP);
            break;
        }
        float dx = 0.0f - me.pos.x;
        float dy = 0.0f - me.pos.y;
        float dist = sqrtf(dx * dx + dy * dy);

        //walls take priority, except right at the base where the walls are close anyway.
        //If something blocks the way home, follow its edge for a while instead of
        //turning straight back into it (that oscillated forever in front of obstacles).
        if (dist > HOME_IGNORE_MM && safety()) {
            homeFollowUntil = millis() + HOME_FOLLOW_MS;
            break;
        }
        if (dist > HOME_IGNORE_MM && millis() < homeFollowUntil) {
            driveExplore();
            break;
        }
        if (dist <= HOME_IGNORE_MM && millis() < recoverMoveUntil) {   //finish a stall recovery
            drive(recoverMoveSpeed, recoverMoveSpeed);
            break;
        }

        float err = wrap180(bearingTo({0.0f, 0.0f}, me) - th);

        if (fabsf(err) > HOME_SPIN_DEG) {
            spin(err > 0 ? 1 : -1, SPIN_SPEED);        //well off: turn on the spot
        } else {
            //proportional steering home + a push away from side walls
            int steer = constrain((int)(HOME_KP * err), -150, 150);
            if (dist > HOME_IGNORE_MM) {
                if (rdg.tL < HOME_SIDE_WARN_MM) steer -= HOME_WALL_PUSH;   //wall left -> right
                if (rdg.tR < HOME_SIDE_WARN_MM) steer += HOME_WALL_PUSH;   //wall right -> left
            }
            int v = (dist < HOME_SLOW_MM) ? HOME_SLOW_SPEED
                                          : speedForFront(frontDist(), CRUISE_SPEED);
            drive(v - steer, v + steer);
        }
        break;
    }

    case R_DROP:
        if (gateDrop()) {
            reset_num_weights();
            turnTarget = START_ON_LEFT ? -INITIAL_TURN_DEG : INITIAL_TURN_DEG;
            enter(R_LEAVE_START);
        } else if (elapsed > 25000UL) {                    //jam safety net
            drive(0, 0);
            reset_num_weights();
            gateStartDrop();
            enter(R_EXPLORE);
        }
        break;
    }
}
