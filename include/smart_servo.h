//Author: Eloise Savry
//Date: 17/7/2026
//Description: This file contains the declarations for the smart servo motor functions.



#ifndef SMART_SERVO_H
#define SMART_SERVO_H

void initSmartServo(void);
void setServoPosition(uint16_t position);
uint16_t getServoPosition(uint16_t &position);
void sortingServoTask();
void startServoSequence(uint16_t position);                   
void sortWantedWeight();
void sortUnwantedWeight();

#endif // SMART_SERVO_H

