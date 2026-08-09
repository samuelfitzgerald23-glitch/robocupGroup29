#include <Arduino.h>
#include "smart_servo.h"
#include <HerkulexServo.h>

// Use the Teensy's Serial7 (pins D28/D29)
HerkulexServoBus servoBus(Serial7);

// Create a servo with ID 1
HerkulexServo servo(servoBus, 1);

//resolution is 0-1023
#define WANTED_WEIGHT 50 //-90 deg
#define UNWANTED_WEIGHT 300 // 90 deg
#define NEUTRAL 140 //0 deg (?)

//range 0 (fastest) to 254 (slowest)
#define SERVO_SPEED 50

//variables for sorting servo sequence (state machine)
volatile bool servoSequenceActive = false;

uint8_t servoState = 0;
uint32_t servoTimer = 0;


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
            if (millis() - servoTimer >= 2000) {
                setServoPosition(WANTED_WEIGHT);
                Serial.println("Moved to wanted weight position");

                servoTimer = millis();
                servoState = 1;
            }
            break;
        case 1:
            if (millis() - servoTimer >= 5000) {
                setServoPosition(NEUTRAL);
                Serial.println("Returned to neutral");

                servoSequenceActive = false;
                servoState = 2;
            }
            break;
        case 2: //finished (could add in rejection of plastic weights here once have colour sensor)
            break;            
    }

    Serial.print("Current position: ");
    Serial.println(getServoPosition());

}

void startServoSequence() 
{
    servoSequenceActive = true;
    servoState = 0; //start at the first state in sorting sequence (waiting for 2 seconds)
    servoTimer = millis();
}
