//Author: Samuel Fitzgerald
//Date: 21/5/2026
//Description: This file contains the declarations for the motor control functions.

#ifndef MOTOR_H
#define MOTOR_H

void initMotors();
void setMotorSpeed(int speed1, int speed2);
void motorTask();

#endif // MOTOR_H