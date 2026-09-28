//Author: Ruby (Team 29)
//Date: 22/9/2026
//Description: Rear payload gate control - see gate.h
//             Gate actuator is a Herkulex DRS-0101 smart servo, "rearServo"
//             (ID 2), daisy-chained on the sorting servo's bus (Serial2, ID 1).
//
//             Behaviour: when the robot enters DROP_PAYLOAD (at the base ~0,0),
//             the gate opens and stays open for GATE_OPEN_MS, then closes -
//             all on its own non-blocking millis() timer. It does NOT hold the
//             state machine or block the scheduler, so the robot's movement and
//             control run untouched: once the gate is open the timer runs
//             independently of what the robot does next.
//
//             ORDER: call initSmartServo() BEFORE initGate() in setup() - the
//             sorting-servo init is what starts Serial2.

#include <Arduino.h>
#include <HerkulexServo.h>

#include "gate.h"
#include "map.h"                 //getMyPOS(), pos_vector_t
#include "State.h"               //getRobotState(), DROP_PAYLOAD

//----------------------------------------------------------------------------
// Config - tune these for your hardware
//----------------------------------------------------------------------------
#define REARSERVO_ID       2         //rear gate servo bus ID (sorting servo is ID 1)

//DRS-0101 raw position is 0..1023. Calibrate these to your physical stops.
#define GATE_CLOSED_POS    000        //raw position with the gate shut
#define GATE_OPEN_POS      289        //raw position with the gate open

#define GATE_SPEED         150         //play-time / speed, 0 (fastest)..254 (slowest)

#define GATE_OPEN_MS       10000UL    //hold the gate open this long when dropping (ms)

#define BASE_TOLERANCE_MM  100.0f     //how close to (0,0) still counts as "at base"
                                      //100 mm == one occupancy-grid cell (CELL_SIZE)

//----------------------------------------------------------------------------
// Servo object
//----------------------------------------------------------------------------
//Shared bus, defined in smart_servo.cpp - both servos live on it, each by ID.
extern HerkulexServoBus servoBus;
static HerkulexServo    rearServo(servoBus, REARSERVO_ID);

//----------------------------------------------------------------------------
// Internal state (non-blocking open timer)
//----------------------------------------------------------------------------

static uint32_t openStart = 0;       //millis() when the gate opened

//----------------------------------------------------------------------------
// Public API
//----------------------------------------------------------------------------
void initGate() {
    //Serial2 is already begun by initSmartServo() - don't begin it again here.
    rearServo.reboot();
    delay(500);
    rearServo.setTorqueOn();

    rearServo.setPosition(GATE_CLOSED_POS, GATE_SPEED, HerkulexLed::Ignore);
    gateOpen = false;
    armed    = true;
}

bool isAtBase() {
    //base is the world-frame origin (0,0). Positions are floats in mm, so they
    //will never be exactly zero - accept anything inside BASE_TOLERANCE_MM.
    pos_vector_t myPOS = getMyPOS();
    return (fabsf(myPOS.pos.x) <= BASE_TOLERANCE_MM) &&
           (fabsf(myPOS.pos.y) <= BASE_TOLERANCE_MM);
}

//Runs the drop sequence: open the gate, hold GATE_OPEN_MS, then close.
//Non-blocking - call repeatedly while in DROP_PAYLOAD. Returns true once the
//gate has closed, so the state machine knows the drop is finished.

static uint32_t openStart = 0;
static uint8_t  dropPhase = 0;         // 0 = open the gate, 1 = holding open

//call once when a drop begins, so a previous (possibly timed-out) drop
//can't leave the sequence half-finished
void gateStartDrop() {
    dropPhase = 0;
}

bool gateDrop() {
    if (dropPhase == 0) {
        rearServo.setPosition(GATE_OPEN_POS, GATE_SPEED, HerkulexLed::Ignore);
        openStart = millis();
        dropPhase = 1;
        return false;
    }
    if (millis() - openStart >= GATE_OPEN_MS) {
        rearServo.setPosition(GATE_CLOSED_POS, GATE_SPEED, HerkulexLed::Ignore);
        dropPhase = 0;
        return true;
    }
    return false;
}

// void gateTest() {
//    servoBus.update();
//
//    static bool freed = false;
//    if (!freed) {
//        rearServo.setTorqueOff();
//        Serial.println("GATE TEST: servo free - push the gate to a stop and read the number");
//        freed = true;
//    }
//
//    static uint32_t last = 0;
//    if (millis() - last >= 250) {
//        last = millis();
//        Serial.print("gate position = ");
//        Serial.println(rearServo.getPosition());
//    }
// }

