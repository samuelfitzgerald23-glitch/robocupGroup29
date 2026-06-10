//Author: Samuel Fitzgerald
//Date: 21/5/2026
//Description: This file contains the declarations for the motor control functions.

#ifndef MOTOR_H
#define MOTOR_H

//initialize motors
void initMotors();

//set drive motor speeds (-450 to 450)
void setDriveSpeed(int speed1, int speed2);

//set fan motor speed (0 to 256)
void setFanSpeed(int speed);

//overall task for motor control
void motorTask();

#endif // MOTOR_H