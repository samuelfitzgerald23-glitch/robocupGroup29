#include <Arduino.h>
#include "smart_servo.h"
#include "proximity.h"
#include "colour_sensor.h"
#include <HerkulexServo.h>

#define SERVO_SECOND 1000000

// Use the Teensy's Serial7 (pins D28/D29)
HerkulexServoBus servoBus(Serial7);

// Create a servo with ID 1
HerkulexServo servo(servoBus, 1);

//setup an interval timer for servo sorting task
IntervalTimer ServoTimeout;
volatile bool servoTimeout = false;


//resolution is 0-1023
#define WANTED_WEIGHT 50 //-90 deg
#define UNWANTED_WEIGHT 300 // 90 deg
#define NEUTRAL 140 //0 deg (?)

//range 0 (fastest) to 254 (slowest)
#define SERVO_SPEED 50

//variables for sorting servo sequence (state machine)
volatile bool servoSequenceActive = false;
uint8_t servoState = 0;
uint16_t targetPosition;

uint8_t num_weights = 0; //keep track of metal weights onboard

void initSmartServo()
{
    Serial7.begin(115200);
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


void servoTask()
{
    servoBus.update();

    //logic for turning one way for 2s then turning the other for 2 seconds
    if (!servoSequenceActive) {
        return;
    }

    switch(servoState)
    {
        case 0:
            if (servoTimeout) {

                servoTimeout = false;

                setServoPosition(targetPosition);
                
                if (targetPosition == WANTED_WEIGHT) {
                    Serial.println("Moved to wanted weight position");
                } else {
                    Serial.println("Moved to unwanted weight position");
                }

                servoState = 1;

                ServoTimeout.begin(servo_timeout, 5*SERVO_SECOND);

                resetMetalDetection();

            }
            break;
        case 1:
            if (servoTimeout) {
                //add logic to not reset to neutral if theres 3 weights onboard to keep (since space may not allow)
                servoTimeout = false;
                
                setServoPosition(NEUTRAL);
                Serial.println("Returned to neutral");

                servoSequenceActive = false;
                servoState = 2;
            }
            break;         

        case 2: //finished
            break;
    }

    Serial.print("Current position: ");
    Serial.println(getServoPosition());

    Serial.print("Number of weights onboard: ");
    Serial.println(num_weights);

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
    servoSequenceActive = true;
    servoState = 0; 

    targetPosition = position;

    if (position == WANTED_WEIGHT) {
        num_weights += 1;
    }

    servoTimeout = false;
    ServoTimeout.begin(servo_timeout, 1*SERVO_SECOND); //starts 2 second timer
}