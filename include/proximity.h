//Author: Eloise Savry
//Date: 15/9/2026
//Description: This file contains the declarations for the proximity sensor functions.

#ifndef PROXIMITY_H
#define PROXIMITY_H

void largeProxISR(void);
void proximity_init(void);
void proximityTask(void);
bool get_metal_weight_detected(void);
void resetMetalDetection(void);

#endif //PROXIMITY_H