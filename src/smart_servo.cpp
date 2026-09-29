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


//resolution of servo is 0-1023
#define WANTED_WEIGHT 600 
#define UNWANTED_WEIGHT 850 
#define NEUTRAL 720 
#define EJECT_MOBILE 500 

//to know if at center or not
#define NEUTRAL_TOLERANCE 20
#define NEUTRAL_RETRY_LIMIT 3 //number of recovery attempts
uint8_t neutralRetries = 0;

//range 0 (fastest) to 254 (slowest)
#define SERVO_SPEED 50

#define METAL_HOLD_TIME  (5 * SERVO_SECOND)
#define REJECTION_HOLD_TIME (3 * SERVO_SECOND)

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
    delay(100);

    // Start at neutral
    setServoPosition(NEUTRAL);

    Serial.println("Initialised smart servo");
}


void setServoPosition(uint16_t position)
{
    servo.setPosition(position, SERVO_SPEED, HerkulexLed::Ignore);
}


uint16_t getServoPosition()
{
    return servo.getPosition();
}

uint32_t getServoHoldTime()
{
    return (targetPosition == WANTED_WEIGHT)
        ? METAL_HOLD_TIME
        : REJECTION_HOLD_TIME;
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
        case 0: // logic for either turning to the unwanted or wanted position (after 1 second wait)
            if (servoTimeout) {
                servoTimeout = false;

                if (targetPosition == EJECT_MOBILE) {

                    setServoPosition(targetPosition);
                    Serial.println("Moved to eject mobile position");
                }

                // Normal wanted/unwanted weight sorting
                if (targetPosition == WANTED_WEIGHT) {

                    if (num_weights < 2) {
                        setServoPosition(targetPosition);
                        Serial.println("Moved to wanted weight position");
                        num_weights++;
                    } else {
                        Serial.println("Holding 3rd metal weight in center");
                        num_weights++;
                    }

                } else if (targetPosition == UNWANTED_WEIGHT) {

                    setServoPosition(targetPosition);
                    Serial.println("Moved to unwanted weight position");
                }

                Serial.print("Number of weights onboard: ");
                Serial.println(num_weights);

                servoState = 1;

                uint32_t holdTime = (targetPosition == WANTED_WEIGHT) //wait longer for metal than plastic+bludger rejections
                    ? METAL_HOLD_TIME
                    : REJECTION_HOLD_TIME;

                ServoTimeout.begin(servo_timeout, holdTime);

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

        case 2: //check it returned to neutral properly

            if (servoTimeout) {
                servoTimeout = false;

                uint16_t actualPosition = getServoPosition();

                if (abs((int)actualPosition - (int)NEUTRAL) <= NEUTRAL_TOLERANCE) {
                    Serial.println("Servo confirmed at neutral");
                    servoSequenceActive = false;
                } else if (neutralRetries < NEUTRAL_RETRY_LIMIT) {
                    neutralRetries++;

                    Serial.println("Neutral blocked; recovery attempt");

                    if (!resetServoAfterOverload()) {
                        Serial.println("Could not restore servo torque");
                        servoSequenceActive = false;
                        break;
                    }

                    setServoPosition(targetPosition);
                    Serial.println("Moving back to original position");

                    servoState = 3;
                    ServoTimeout.begin(servo_timeout, getServoHoldTime());
                

                    
                } else {
                    Serial.println("Servo failed to return to neutral; check for jam");

                    servo.setTorqueOn();
                    setServoPosition(targetPosition);
                    servoSequenceActive = false;
                }
            }
            break;
            
        case 3: // recovery: wait at wanted position, then retry neutral (when it switches back to state 2)
            if (servoTimeout) {
                servoTimeout = false;
                
                //ervo.setTorqueOn();
                setServoPosition(NEUTRAL);
                Serial.println("Retrying neutral after recovery move");

                servoState = 2;
                ServoTimeout.begin(servo_timeout, 3 * SERVO_SECOND);
            }
            break;
    }
}


void sortWantedWeight()
{
    startServoSequence(WANTED_WEIGHT);
}

void sortUnwantedWeight()
{
    startServoSequence(UNWANTED_WEIGHT);
}

void ejectMobileElement()
{
    startServoSequence(EJECT_MOBILE);
}

bool resetServoAfterOverload()
{
    HerkulexStatusError error;
    HerkulexStatusDetail detail;

    servo.getStatus(error, detail);

    if ((uint8_t)error & 0x10) {
        Serial.println("Servo overload detected - resetting");

        // Clear overload status
        servo.writeRam(HerkulexRamRegister::StatusError, 0x00);

        // Restore torque
        servo.setTorqueOn();

        // Check that torque has actually returned
        servo.getStatus(error, detail);

        if ((uint8_t)detail & 0x40) {
            Serial.println("Servo torque restored");
            return true;
        }

        Serial.println("Servo torque failed to restore");
        return false;
    }

    return true;
}

void startServoSequence(uint16_t position) 
{

    if (servoSequenceActive) {
        //Serial.println("Sort request ignored: servo is already busy");
        return;
    }

    cancelMobileDetection();

    servoSequenceActive = true;
    servoState = 0; 
    targetPosition = position;

    servoTimeout = false;
    ServoTimeout.begin(servo_timeout, 1*SERVO_SECOND); // wait 1 second before sorting
}



//this test function lets you turn the servo to see its output position through serial monitor
void servo_test()
{
    servoBus.update();

    static bool torqueDisabled = false;

    if (!torqueDisabled) {
        servo.setTorqueOff();
        torqueDisabled = true;
        Serial.println("Servo torque OFF - move it by hand");
    }

    uint16_t currentPosition = getServoPosition();

    Serial.print("Current position: ");
    Serial.println(currentPosition);
}


bool isServoSequenceActive(void)
{
    return servoSequenceActive;
}

uint8_t get_num_weights(void)
{
    return num_weights;
}

void reset_num_weights(void)
{
    num_weights = 0;
}

