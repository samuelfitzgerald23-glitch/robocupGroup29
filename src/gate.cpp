//Author: Ruby (Team 29)
//Date: 22/9/2026
//Description: Rear payload gate control - see gate.h
//             Herkulex DRS-0101 "rearServo" (ID 2) on the sorting servo's bus.
//             During DROP_PAYLOAD: shake to dislodge weights + open the gate,
//             hold GATE_OPEN_MS, then close - all non-blocking.
//             ORDER: call initSmartServo() BEFORE initGate() in setup().

#include <Arduino.h>
#include <HerkulexServo.h>

#include "gate.h"
#include "map.h"                 //getMyPOS(), pos_vector_t, pos_t
#include "State.h"               //getRobotState(), DROP_PAYLOAD
#include "motor.h"               //setDriveSpeed()

//----------------------------------------------------------------------------
// Config
//----------------------------------------------------------------------------
#define REARSERVO_ID       2         //rear gate servo bus ID (sorting servo is ID 1)

#define GATE_CLOSED_POS    0         //raw position with the gate shut (0..1023)
#define GATE_OPEN_POS      289       //raw position with the gate open
#define GATE_SPEED         150       //play-time / speed, 0 (fastest)..254 (slowest)
#define GATE_OPEN_MS       10000UL   //hold the gate open this long (ms)

#define SHAKE_SPEED        400       //shake drive speed (-450..450)
#define SHAKE_STEP_MS      120       //ms per shake move (smaller = sharper)
#define SHAKE_MAX_MM       200.0f    //stop shaking if we drift this far from base

//----------------------------------------------------------------------------
// Servo object + state
//----------------------------------------------------------------------------
extern HerkulexServoBus servoBus;                    //defined in smart_servo.cpp
static HerkulexServo    rearServo(servoBus, REARSERVO_ID);

static uint32_t openStart = 0;       //millis() when the gate opened
static uint8_t  dropPhase = 0;       //0 = open the gate, 1 = holding open

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
    //bounds check: if we've drifted too far from the base origin, stop
    if (fabsf(p.x) > SHAKE_MAX_MM || fabsf(p.y) > SHAKE_MAX_MM) {
        setDriveSpeed(0, 0);
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
// Drop sequence: shake + open -> hold -> close. Returns true once closed.
//----------------------------------------------------------------------------
bool gateDrop() {
    shakeRobot();                  // shake to dislodge weights (bounded to base)

    if (dropPhase == 0) {
        rearServo.setPosition(GATE_OPEN_POS, GATE_SPEED, HerkulexLed::Ignore);
        openStart = millis();
        dropPhase = 1;
        return false;
    }
    if (millis() - openStart >= GATE_OPEN_MS) {
        rearServo.setPosition(GATE_CLOSED_POS, GATE_SPEED, HerkulexLed::Ignore);
        setDriveSpeed(0, 0);       // stop the shake
        dropPhase = 0;
        return true;
    }
    return false;
}

//----------------------------------------------------------------------------
// TEST: loops a full drop so you can watch shake + gate on the bench.
// Temporarily point tState_task at this instead of StateMachineTask.
//----------------------------------------------------------------------------
void gateDropTest() {
    static uint8_t  tphase     = 0;     // 0 = running a drop, 1 = pause between
    static uint32_t pauseStart = 0;
    static uint32_t lastPrint  = 0;

    if (tphase == 0) {
        if (gateDrop()) {
            setDriveSpeed(0, 0);
            Serial.println("TEST: drop complete (gate closed) - pausing 3 s");
            pauseStart = millis();
            tphase = 1;
        } else if (millis() - lastPrint >= 500) {
            lastPrint = millis();
            pos_t p = getMyPOS().pos;
            Serial.print("TEST: dropping...  pos x=");
            Serial.print(p.x);
            Serial.print(" y=");
            Serial.println(p.y);
        }
    } else {
        if (millis() - pauseStart >= 3000) {
            Serial.println("TEST: starting next drop cycle");
            gateStartDrop();
            tphase = 0;
        }
    }
}
