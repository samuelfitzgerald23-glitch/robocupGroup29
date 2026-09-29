//Author: Ruby (Team 29)
//Date: 29/9/2026
//Description: Rear payload gate control - see gate.h
//             Herkulex DRS-0101 "rearServo" (ID 2) on the sorting servo's bus.
//             DROP_PAYLOAD sequence: open gate + shake for SHAKE_MS, then sit
//             still for STILL_MS (gate stays open), then close. All non-blocking.
//             ORDER: call initSmartServo() BEFORE initGate() in setup().

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

#define SHAKE_MS           10000UL   //shake this long (gate open)
#define STILL_MS           10000UL   //then sit still this long (gate open), then close

#define SHAKE_SPEED        400       //shake drive speed (-450..450)
#define SHAKE_STEP_MS      120       //ms per shake move (smaller = sharper)
#define SHAKE_MAX_MM       200.0f    //stop shaking if we drift this far from base

//----------------------------------------------------------------------------
// Servo object + state
//----------------------------------------------------------------------------
extern HerkulexServoBus servoBus;                    //defined in smart_servo.cpp
static HerkulexServo    rearServo(servoBus, REARSERVO_ID);

static uint32_t openStart = 0;       //millis() when the gate opened
static uint8_t  dropPhase = 0;       //0 = open, 1 = open+shaking, 2 = open+still

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

//call once when a drop begins, so a previous (possibly timed-out) drop
//can't leave the sequence half-finished
void gateStartDrop() {
    dropPhase = 0;
}

//----------------------------------------------------------------------------
// Shake helper - reads its own position, drives its own motors (no args)
//----------------------------------------------------------------------------
static void shakeRobot() {
    pos_t p = getMyPOS().pos;
    if (fabsf(p.x) > SHAKE_MAX_MM || fabsf(p.y) > SHAKE_MAX_MM) {
        setDriveSpeed(0, 0);         //drifted off base - stop
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
// Drop: open -> shake SHAKE_MS -> sit still STILL_MS -> close. True when done.
//----------------------------------------------------------------------------
bool gateDrop() {
    switch (dropPhase) {
        case 0:  // open the gate, start the clock
            rearServo.setPosition(GATE_OPEN_POS, GATE_SPEED, HerkulexLed::Ignore);
            openStart = millis();
            dropPhase = 1;
            return false;

        case 1:  // shake for SHAKE_MS, then CLOSE immediately
        if (millis() - openStart < SHAKE_MS) {
            shakeRobot();
        } else {
            setDriveSpeed(0, 0);                                          // stop shaking
            rearServo.setPosition(GATE_CLOSED_POS, GATE_SPEED, HerkulexLed::Ignore);  // close NOW
            dropPhase = 2;
        }
        return false;

        case 2:  // gate already shut - just sit still for STILL_MS
            if (millis() - openStart >= SHAKE_MS + STILL_MS) {
                dropPhase = 0;
                return true;                   // drop complete
            }
            return false;
    return false;
}

//----------------------------------------------------------------------------
// TEST: loops a full drop so you can watch shake + gate on the bench.
// Temporarily run this from tGate_task (with tState_task disabled).
//----------------------------------------------------------------------------
void gateDropTest() {
    static bool done = false;
    if (done) return;                          // one cycle only - do nothing after

    if (gateDrop()) {
        setDriveSpeed(0, 0);
        Serial.println("TEST: drop complete - stopping (one cycle)");
        done = true;
    }
}
