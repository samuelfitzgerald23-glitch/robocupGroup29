//Author: Eloise Savry
//Date: 17/7/2026
//Description: This file contains the declarations for the smart servo motor functions.



#ifndef SMART_SERVO_H
#define SMART_SERVO_H

void initSmartServo(void);
void setServoPosition(uint16_t position);
uint16_t getServoPosition(void);
void sortingServoTask(void);
void startServoSequence(uint16_t position);                   
void sortWantedWeight(void);
void sortUnwantedWeight(void);
bool resetServoAfterOverload(void);
void ejectMobileElement(void);
void servo_test(void);
bool isServoSequenceActive(void);
uint8_t get_num_weights(void);

#endif // SMART_SERVO_H

