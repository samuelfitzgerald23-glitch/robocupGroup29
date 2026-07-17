#include <Arduino.h>
#include "smart_servo.h"
#include <HerkulexServo.h>

// Use the Teensy's Serial7 (pins D28/D29)
HerkulexServoBus servoBus(Serial7);

// Create a servo with ID 1
HerkulexServo servo(servoBus, 1);

//resolution is 0-1023
#define WANTED_WEIGHT 235 //-90 deg
#define UNWANTED_WEIGHT 789 // 90 deg
#define NEUTRAL 512 //0 deg (?)

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
    servo.setPosition(position, 0, HerkulexLed::Ignore);
}

uint16_t getServoPosition()
{
    return servo.getPosition();
}

void servoTask()
{
    uint16_t currentPosition;
    
    servoBus.update();

    setServoPosition(WANTED_WEIGHT);
    currentPosition = getServoPosition();
    Serial.print("Current position: ");
    Serial.println(currentPosition);

    delay(2000);

    setServoPosition(NEUTRAL);
    currentPosition = getServoPosition();
    Serial.print("Current position: ");
    Serial.println(currentPosition);

    delay(2000);

    setServoPosition(UNWANTED_WEIGHT);
    currentPosition = getServoPosition();
    Serial.print("Current position: ");
    Serial.println(currentPosition);

    //logic for moving to specific position depending on proximity sensor output
}
