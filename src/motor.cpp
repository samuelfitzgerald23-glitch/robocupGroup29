#include <Servo.h>
#include "motor.h"

#define MOTOR1_PIN 0    //left
#define MOTOR2_PIN 1    //right

#define FULLREVERSE 1050
#define STOP 1500
#define FULLFORWARD 1950

// create servo objects for motors
static Servo Motor1;
static Servo Motor2;

void initMotors() {
    // Assign motor pins to servo objects
    Motor1.attach(MOTOR1_PIN);
    Motor2.attach(MOTOR2_PIN);

    // Stop motors initially
    Motor1.writeMicroseconds(STOP);
    Motor2.writeMicroseconds(STOP);
}

void setMotorSpeed(int speed1, int speed2) {
    int pulseWidth1 = STOP - speed1;
    int pulseWidth2 = STOP + speed2;

    if (pulseWidth1 > FULLFORWARD) pulseWidth1 = FULLFORWARD;
    if (pulseWidth1 < FULLREVERSE) pulseWidth1 = FULLREVERSE;

    if (pulseWidth2 > FULLFORWARD) pulseWidth2 = FULLFORWARD;
    if (pulseWidth2 < FULLREVERSE) pulseWidth2 = FULLREVERSE;

    Motor1.writeMicroseconds(pulseWidth1);
    Motor2.writeMicroseconds(pulseWidth2);
}

void motorTask() {
    setMotorSpeed(450, 450);
}