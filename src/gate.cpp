//Author: Ruby (Team 29)
//Date: 29/9/2026
//Description: Rear payload gate control - see gate.h
//             Herkulex DRS-0101 "rearServo" (ID 2) on the sorting servo's bus.
//             DROP_PAYLOAD sequence: open gate + shake for SHAKE_MS, then close
//             and sit still for STILL_MS. Non-blocking; runs from the state
//             machine's DROP_PAYLOAD case via gateDrop().
//             ORDER: call initSmartServo() BEFORE initGate() in setup().
//
//Changes (reactive docking):
//  - shake limit is measured from where the drop STARTED, not from the origin
//    (the pose estimate drifts, so "distance from origin" stopped the shake)
//  - the close command is re-sent every GATE_RESEND_MS during the still phase,
//    after clearing any servo overload fault, so one lost packet can't leave
//    the gate open
//  - STILL_MS lengthened so the gate has time to finish closing

#include <Arduino.h>
#include <HerkulexServo.h>

#include "gate.h"
#include "map.h"                 //getMyPOS(), pos_t
#include "State.h"               //getRobotState(), DROP_PAYLOAD
#include "motor.h"               //setDriveSpeed()

//----------------------------------------------------------------------------
// Config
//----------------------------------------------------------------------------
#define REARSERVO_ID       2         //rear gate servo bus ID (sorting servo is ID 1)

#define GATE_CLOSED_POS    0         //raw position with the gate shut (0..1023)
#define GATE_OPEN_POS      289       //raw position with the gate open
#define GATE_SPEED         150       //play-time / speed, 0 (fastest)..254 (slowest)
#define GATE_RESEND_MS     250       //re-send the close command this often while shut-and-still

#define SHAKE_MS           5000UL    //shake this long (gate open)
#define STILL_MS           3000UL    //sit still this long after closing (gate travel takes ~1.7 s)

#define SHAKE_SPEED        400       //shake drive speed (motor.cpp clamps to +/-350)
#define SHAKE_STEP_MS      120       //ms per shake move (smaller = sharper)
#define SHAKE_MAX_MM       200.0f    //stop shaking if we drift this far from where the drop STARTED

#define BASE_TOLERANCE_MM  50.0f     //how close to (0,0) counts as "at base"


//----------------------------------------------------------------------------
// Servo object + state
//----------------------------------------------------------------------------
extern HerkulexServoBus servoBus;                    //defined in smart_servo.cpp
static HerkulexServo    rearServo(servoBus, REARSERVO_ID);

static uint32_t openStart = 0;       //millis() when the gate opened
static uint8_t  dropPhase = 0;       //0 = open, 1 = open+shaking, 2 = shut+still
static float    dropX = 0, dropY = 0;   //where the robot was when the drop began
static uint32_t lastCloseCmd = 0;

//----------------------------------------------------------------------------
// Setup
//----------------------------------------------------------------------------
void initGate() {
    //Serial2 is already begun by initSmartServo() - don't begin it again here.
    rearServo.reboot();
    delay(500);
    rearServo.setTorqueOn();
    rearServo.setPosition(GATE_CLOSED_POS, GATE_SPEED, HerkulexLed::Ignore);
    dropPhase = 0;
}

//----------------------------------------------------------------------------
// True when the robot is parked at the base (world-frame origin) within
// BASE_TOLERANCE_MM. Used by the state machine to confirm a real arrival
// before dropping.
//----------------------------------------------------------------------------
bool isAtBase() {
    pos_t p = getMyPOS().pos;
    return (fabsf(p.x) <= BASE_TOLERANCE_MM) &&
           (fabsf(p.y) <= BASE_TOLERANCE_MM);
}

//call once when a drop begins, so a previous (possibly timed-out) drop
//can't leave the sequence half-finished
void gateStartDrop() {
    dropPhase = 0;
}

//----------------------------------------------------------------------------
// Clear a servo overload fault (e.g. from pushing against weights) and make
// sure torque is on, so the next command is actually obeyed
//----------------------------------------------------------------------------
static void recoverRearServo() {
    HerkulexStatusError error;
    HerkulexStatusDetail detail;

    rearServo.getStatus(error, detail);
    if ((uint8_t)error & 0x10) {                         //overload
        Serial.println("Gate servo overload - clearing");
        rearServo.writeRam(HerkulexRamRegister::StatusError, 0x00);
    }
    rearServo.setTorqueOn();
}

//----------------------------------------------------------------------------
// Shake helper - drives its own motors. Limit is measured from where the
// drop started, so pose drift relative to the origin doesn't matter.
//----------------------------------------------------------------------------
static void shakeRobot() {
    pos_t p = getMyPOS().pos;
    if (fabsf(p.x - dropX) > SHAKE_MAX_MM || fabsf(p.y - dropY) > SHAKE_MAX_MM) {
        setDriveSpeed(0, 0);         //drifted away from where we started - stop
        return;
    }

    static const int pattern[][2] = {   // {arg1, arg2} for setDriveSpeed()
        {  SHAKE_SPEED,  SHAKE_SPEED },  // forward
        { -SHAKE_SPEED, -SHAKE_SPEED },  // back
        {  SHAKE_SPEED,  SHAKE_SPEED },  // forward
        { -SHAKE_SPEED, -SHAKE_SPEED },  // back
        { -SHAKE_SPEED,  SHAKE_SPEED },  // spin one way
        {  SHAKE_SPEED, -SHAKE_SPEED },  // spin the other way
    };
    static const int steps = sizeof(pattern) / sizeof(pattern[0]);
    static uint32_t lastStep = 0;
    static int idx = 0;

    if (millis() - lastStep >= SHAKE_STEP_MS) {
        lastStep = millis();
        setDriveSpeed(pattern[idx][0], pattern[idx][1]);
        idx = (idx + 1) % steps;
    }
}

//----------------------------------------------------------------------------
// Drop: open -> shake SHAKE_MS -> close -> sit still STILL_MS. True when done.
//----------------------------------------------------------------------------
bool gateDrop() {
    switch (dropPhase) {
        case 0: {  // open the gate, remember where we are, start the clock
            pos_t p = getMyPOS().pos;
            dropX = p.x;
            dropY = p.y;
            rearServo.setPosition(GATE_OPEN_POS, GATE_SPEED, HerkulexLed::Ignore);
            openStart = millis();
            dropPhase = 1;
            return false;
        }

        case 1:  // shake for SHAKE_MS, then close immediately
            if (millis() - openStart < SHAKE_MS) {
                shakeRobot();
            } else {
                setDriveSpeed(0, 0);                                          // stop shaking
                recoverRearServo();
                rearServo.setPosition(GATE_CLOSED_POS, GATE_SPEED, HerkulexLed::Ignore);  // close now
                lastCloseCmd = millis();
                dropPhase = 2;
            }
            return false;

        case 2:  // gate shut - keep telling it to stay shut, sit still for STILL_MS, then finish
            setDriveSpeed(0, 0);

            if (millis() - lastCloseCmd >= GATE_RESEND_MS) {
                lastCloseCmd = millis();
                recoverRearServo();
                rearServo.setPosition(GATE_CLOSED_POS, GATE_SPEED, HerkulexLed::Ignore);
            }

            if (millis() - openStart >= SHAKE_MS + STILL_MS) {
                Serial.print("Gate finished, position = ");
                Serial.println(rearServo.getPosition());    //should be close to GATE_CLOSED_POS
                dropPhase = 0;
                return true;                   // drop complete
            }
            return false;
    }
    return false;
}