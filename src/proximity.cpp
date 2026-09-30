#include <Arduino.h>
#include "proximity.h"
#include "colour_sensor.h"
#include "smart_servo.h"


#define LARGE_PROX_OUTPUT 20

#define WEIGHT_CONFIRM_DELAY 1000  // 1 second

volatile bool largeProxTriggered = false;
volatile bool metalWeightDetected = false;

// Non-blocking confirmation state
static bool waitingForConfirmation = false;
static uint32_t confirmationStartTime = 0;

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

void proximityTask()
{
    // Step 1: Start the confirmation timer when proximity triggers
    if (largeProxTriggered) {

        // Clear the ISR flag safely
        noInterrupts();
        largeProxTriggered = false;
        interrupts();

        // Only start a new timer if we aren't already waiting
        if (!waitingForConfirmation && !metalWeightDetected) {

            waitingForConfirmation = true;
            confirmationStartTime = millis();

            Serial.println("Proximity triggered - waiting to confirm weight");
        }
    }

    // Step 2: Check whether the wait has finished
        if (waitingForConfirmation) {

            if (millis() - confirmationStartTime >= WEIGHT_CONFIRM_DELAY) {

                waitingForConfirmation = false;

                // The proximity output is active-low; require it to remain active after the delay.
                if (digitalRead(LARGE_PROX_OUTPUT) == LOW) {

                    metalWeightDetected = true;

                    Serial.println("Metal weight detected!");

                    sortWantedWeight();

                } else {
                    Serial.println("False proximity trigger ignored");
                }
            }
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

