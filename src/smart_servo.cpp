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
#define WANTED_WEIGHT 50 
#define UNWANTED_WEIGHT 300 
#define NEUTRAL 150 
#define EJECT_MOBILE 10 //calbrate this???

#define MOBILE_EJECT_SPEED -200
#define MOBILE_EJECT_DURATION (2 * SERVO_SECOND)
#define MOBILE_EJECT_WAIT (5 * SERVO_SECOND)
#define MOBILE_NEUTRAL_WAIT (1 * SERVO_SECOND)

//to know if at center or not
#define NEUTRAL_TOLERANCE 20
#define NEUTRAL_RETRY_LIMIT 3 //how many times to try unblock stuck metal weight 
uint8_t neutralRetries = 0;

//range 0 (fastest) to 254 (slowest)
#define SERVO_SPEED 70

//variables for sorting servo sequence (state machine)
volatile bool servoSequenceActive = false;
uint8_t servoState = 0;
uint16_t targetPosition;

uint8_t num_weights = 0; //keep track of metal weights onboard

//check whether to reverse the fans
static bool reverse_fans = false;


void initSmartServo()
{
    SERIAL_SERVO.begin(115200);
    delay(100);
    servo.reboot();
    delay(500);
    
    // Initialise in position control mode
    servo.setTorqueOff();
    delay(100);

    servo.enablePositionControlMode();
    delay(100);

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

                if (targetPosition == EJECT_MOBILE) {

                    // Start reversing fans
                    reverse_fans = true;

                    // Switch to speed mode and begin clockwise rotation
                    servo.enableSpeedControlMode();
                    servo.setSpeed(MOBILE_EJECT_SPEED, 0);

                    Serial.println("Starting mobile element ejection");

                    // Move to dedicated mobile ejection state
                    servoState = 4;
                    ServoTimeout.begin(
                        servo_timeout,
                        MOBILE_EJECT_DURATION
                    );

                    resetMetalDetection();
                    break;
                }

                // Normal wanted/unwanted weight sorting
                if (targetPosition == WANTED_WEIGHT) {

                    if (num_weights < 2) {
                        setServoPosition(targetPosition);
                        Serial.println("Moved to wanted weight position");
                        num_weights++;
                    } else {
                        Serial.println("Holding 3rd metal weight in center");
                    }

                } else if (targetPosition == UNWANTED_WEIGHT) {

                    setServoPosition(targetPosition);
                    Serial.println("Moved to unwanted weight position");
                }

                Serial.print("Number of weights onboard: ");
                Serial.println(num_weights);

                servoState = 1;
                ServoTimeout.begin(servo_timeout, 5 * SERVO_SECOND);

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

                if (abs((int)actualPosition - (int)NEUTRAL)
                        <= NEUTRAL_TOLERANCE) {

                    Serial.println("Servo confirmed at neutral");
                    servoSequenceActive = false;

                } else if (neutralRetries < NEUTRAL_RETRY_LIMIT) {

                    neutralRetries++;

                    setServoPosition(WANTED_WEIGHT);
                    Serial.println(
                        "Neutral blocked; moving back to wanted position"
                    );

                    servoState = 3;
                    ServoTimeout.begin(
                        servo_timeout,
                        3 * SERVO_SECOND
                    );

                } else {

                    Serial.println(
                        "Servo failed to return to neutral; check for jam"
                    );

                    setServoPosition(WANTED_WEIGHT);
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

        case 4: // Stop after clockwise ejection
            if (servoTimeout) {
                servoTimeout = false;

                servo.setSpeed(0, 0);

                Serial.println("Ejection rotation stopped");

                servoState = 5;
                ServoTimeout.begin(
                    servo_timeout,
                    MOBILE_EJECT_WAIT
                );
            }
            break;
        case 5: // Return to neutral position mode
            if (servoTimeout) {
                servoTimeout = false;

                servo.setSpeed(0, 0);

                servo.setTorqueOff();
                servo.enablePositionControlMode();
                servo.setTorqueOn();
                
                //Serial.print("Current position: ");
                //Serial.println(getServoPosition());

                setServoPosition(NEUTRAL);

                Serial.println("Returning to neutral");

                servoState = 6;
                ServoTimeout.begin(
                    servo_timeout,
                    MOBILE_NEUTRAL_WAIT
                );
            }
            break;
        case 6: // Check whether mobile element cleared
            if (servoTimeout) {
                servoTimeout = false;

                if (get_mobile_element_present()) {

                    Serial.println(
                        "Mobile element still present - repeating"
                    );

                    // Switch back to speed mode
                    servo.enableSpeedControlMode();
                    servo.setSpeed(MOBILE_EJECT_SPEED, 0);

                    servoState = 4;
                    ServoTimeout.begin(
                        servo_timeout,
                        MOBILE_EJECT_DURATION
                    );

                } else {

                    Serial.println(
                        "Mobile element cleared - fans forward"
                    );

                    reverse_fans = false;
                    servoSequenceActive = false;
                }
            }
            break;
    }

    Serial.print("Current position: ");
    Serial.println(getServoPosition());

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

bool getReverseFans(void)
{
    return reverse_fans;
}

// void servo_test()
// {
//     servoBus.update();

//     static bool torqueDisabled = false;

//     if (!torqueDisabled) {
//         servo.setTorqueOff();
//         torqueDisabled = true;
//         Serial.println("Servo torque OFF - move it by hand");
//     }

//     uint16_t currentPosition = getServoPosition();

//     Serial.print("Current position: ");
//     Serial.println(currentPosition);
// }


// void servo_test() 

// { 
//     servoBus.update(); 
//     uint16_t currentPosition = getServoPosition(); 
//     Serial.print("current pos:"); 
//     Serial.println(currentPosition); 
//     setServoPosition(EJECT_MOBILE); 

//     // if (currentPosition != WANTED_WEIGHT) { 
//     // setServoPosition(WANTED_WEIGHT); 
//     // } 
// }


// void servo_test()
// {
//     servoBus.update();

//     servo.setSpeed(-100, 100);
// }


void servo_test()
{    
    Serial.print("inside servo test");

    static bool moving = false;
    static bool done = false;
    static unsigned long startTime = 0;

    servoBus.update();

    if (done) {
        return;  // Finished - never start again
    }

    if (!moving) {
        Serial.println("Starting 3-second clockwise movement");

        servo.setSpeed(-200, 0);  // go clockwise for bludger position
        startTime = millis();
        moving = true;
    }

    if (moving && millis() - startTime >= 3000) {
        servo.setSpeed(0, 0);     // Stop
        moving = false;
        done = true;
        
        Serial.print("sServo movement finished");
    }
}
