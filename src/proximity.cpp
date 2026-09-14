#include <Arduino.h>
#include "smart_servo.h"


#define LARGE_PROX_OUTPUT 20

volatile bool largeProxTriggered = false;
volatile bool metalWeightDetected = false; 

/*
Proximity sensor init/task functions
*/
void largeProxISR()
{
    largeProxTriggered = true;
    metalWeightDetected = true;
}


void proximity_init()
{
    pinMode(LARGE_PROX_OUTPUT, INPUT);

    //make proximity sensor generate an interrupt if weight detected
    attachInterrupt(
      digitalPinToInterrupt(LARGE_PROX_OUTPUT),
      largeProxISR,
      FALLING
    );

}


void proximity_task()
{
    if (largeProxTriggered) {
      largeProxTriggered = false;
      Serial.println("Large proximity triggered!");
      
      sortWantedWeight();
    }
}

bool get_metal_weight_detected()
{
    return metalWeightDetected;
}

void resetMetalDetection()
{
    metalWeightDetected = false;
}