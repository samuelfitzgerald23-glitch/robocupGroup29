//Author: Ruby (Team 29)
//Date: 20/9/2026
//Description: This file contains the declarations for the rear payload gate.
//             The gate (Herkulex DRS-0101 smart servo) opens only when the robot
//             is at the base (world frame ~ 0,0) AND in the DROP_PAYLOAD state.

#ifndef GATE_H
#define GATE_H

//initialise the gate servo (bus + torque)
void initGate();

//true when the robot is parked at the base (0,0) within tolerance
bool isAtBase();

bool gateDrop();          // was: void gateTask();

//temporary: position tuning helper (see gate.cpp)
void gateTest();

#endif // GATE_H
