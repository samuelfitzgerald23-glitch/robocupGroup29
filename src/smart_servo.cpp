#include <Arduino.h>
#include "smart_servo.h"
#include "proximity.h"
#include "colour_sensor.h"
#include <HerkulexServo.h>

#define SERVO_SECOND 1000000
#define SERIAL_SERVO Serial2

// Use the Teensy's Serial7 (pins D28/D29)
HerkulexServoBus servoBus(SERIAL_SERVO);

// Create a servo with ID 1
HerkulexServo servo(servoBus, 1);

//setup an interval timer for servo sorting task
IntervalTimer ServoTimeout;
volatile bool servoTimeout = false;


//resolution is 0-1023
#define WANTED_WEIGHT 50 //-90 deg
#define UNWANTED_WEIGHT 300 // 90 deg
#define NEUTRAL 150 //0 deg (?)

//to know if at center or not
#define NEUTRAL_TOLERANCE 20
#define NEUTRAL_RETRY_LIMIT 3
uint8_t neutralRetries = 0;

//range 0 (fastest) to 254 (slowest)
#define SERVO_SPEED 50

//variables for sorting servo sequence (state machine)
volatile bool servoSequenceActive = false;
uint8_t servoState = 0;
uint16_t targetPosition;

uint8_t num_weights = 0; //keep track of metal weights onboard

void initSmartServo()
{
    SERIAL_SERVO.begin(115200);
    delay(100);
    servo.reboot();
    delay(500);
    servo.setTorqueOn();

    setServoPosition(NEUTRAL);

    Serial.print("Inisitialised smart servo ");
}


void setServoPosition(uint16_t position)
{
    servo.setPosition(position, SERVO_SPEED, HerkulexLed::Ignore);
}


uint16_t getServoPosition()
{
    return servo.getPosition();
}


void servo_timeout()
{
    ServoTimeout.end();
    servoTimeout = true;
}

// 
void sortingServoTask()
{
    servoBus.update();

    if (!servoSequenceActive) {
        return;
    }

    switch(servoState)
    {
        case 0: // logic fdor either turning to the unwanted or wanted position (after 2 seconds)
            if (servoTimeout) {

                servoTimeout = false;

                if ((num_weights < 2)&&(targetPosition == WANTED_WEIGHT)) {
                    setServoPosition(targetPosition);
                    Serial.println("Moved to wanted weight position");
                    num_weights += 1;
                } else if ((num_weights >= 2)&&(targetPosition == WANTED_WEIGHT))  {
                    Serial.println("Holding 3rd metal weight in center");
                } else {
                    setServoPosition(targetPosition);
                    Serial.println("Moved to unwanted weight position");
                }

                Serial.print("Number of weights onboard: ");
                Serial.println(num_weights);

                servoState = 1;

                ServoTimeout.begin(servo_timeout, 5*SERVO_SECOND);

                resetMetalDetection();

            }
            break;
        case 1: //logic for returning to center position
            if (servoTimeout) {
                servoTimeout = false;
                
                setServoPosition(NEUTRAL);
                servoState = 2;
                neutralRetries = 0;
                ServoTimeout.begin(servo_timeout, 1 * SERVO_SECOND); //set another 1 second timer
        
            }
            break;         

        case 2: //finished
            if (servoTimeout) {
                servoTimeout = false;

                uint16_t actualPosition = getServoPosition();

                if (abs((int)actualPosition - (int)NEUTRAL) <= NEUTRAL_TOLERANCE) {
                    Serial.println("Servo confirmed at neutral");
                    servoSequenceActive = false;
                } else if (neutralRetries < NEUTRAL_RETRY_LIMIT) {
                    neutralRetries++;

                    // Move away from the jam first.
                    setServoPosition(WANTED_WEIGHT);
                    Serial.println("Neutral blocked; moving back to wanted position");

                    servoState = 3;
                    ServoTimeout.begin(servo_timeout, 3 * SERVO_SECOND); // wait for another 3 seconds before retrying to go to center
                }
                else {
                    Serial.println("Servo failed to return to neutral; check for jam");
                    servoSequenceActive = false;
                }
            }
            break;

        case 3: // recovery: wait at wanted position, then retry neutral (when it switches back to state 2)
            if (servoTimeout) {
                servoTimeout = false;

                setServoPosition(NEUTRAL);
                Serial.println("Retrying neutral after recovery move");

                servoState = 2;
                ServoTimeout.begin(servo_timeout, 3 * SERVO_SECOND);
            }
            break;
    }

    //Serial.print("Current position: ");
    //Serial.println(getServoPosition());

}

void sortWantedWeight()
{
    startServoSequence(WANTED_WEIGHT);
}

void sortUnwantedWeight()
{
    startServoSequence(UNWANTED_WEIGHT);
}


void startServoSequence(uint16_t position) 
{

    if (servoSequenceActive) {
        Serial.println("Sort request ignored: servo is already busy");
        return;
    }

    servoSequenceActive = true;
    servoState = 0; 
    targetPosition = position;

    servoTimeout = false;
    ServoTimeout.begin(servo_timeout, 1*SERVO_SECOND); // wait 2 seconds before sorting
}

void servo_test()
{
    servoBus.update();

    uint16_t currentPosition = getServoPosition();

    if (currentPosition != WANTED_WEIGHT) {
        setServoPosition(WANTED_WEIGHT);
    }

}