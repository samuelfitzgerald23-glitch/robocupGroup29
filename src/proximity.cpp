#include <Arduino.h>
#include "proximity.h"
#include "colour_sensor.h"
#include "smart_servo.h"


#define LARGE_PROX_OUTPUT 20

volatile bool largeProxTriggered = false;
volatile bool metalWeightDetected = false;

#define METAL_CLEAR_MIN 900
#define METAL_CLEAR_MAX 2700


/*
Proximity sensor init/task functions
*/
void largeProxISR()
{
    largeProxTriggered = true;
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
    if (!largeProxTriggered) {
        return;
    }

    largeProxTriggered = false;

    uint16_t clear = get_clear_value();
    Serial.print("Clear: ");
    Serial.println(clear);

    if (clear>=METAL_CLEAR_MIN && clear <= METAL_CLEAR_MAX) {
      metalWeightDetected = true;
      Serial.println("Metal weight detected!");
      sortWantedWeight();
    } else {
        Serial.println("False proximity trigger ignored");
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

