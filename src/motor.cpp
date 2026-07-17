#include <Servo.h>
#include "motor.h"
#include "smart_servo.h"
#include <Arduino.h>

//driving motors
#define DRIVE1_PIN 0    //left
#define DRIVE2_PIN 1    //right

//fan blade motors
#define FAN1_PIN 7 //RHS
#define FAN2_PIN 8 //LHS
#define FAN_OFFSET 180

//some change to see if it pushes to correct branch


//motor speed limits
#define FULLREVERSE 1050
#define STOP 1500
#define FULLFORWARD 1950

// create servo objects for motors
static Servo Drive1;
static Servo Drive2;
static Servo Fan1;
static Servo Fan2;

void initMotors() {
    // Assign motor pins to servo objects
    Drive1.attach(DRIVE1_PIN);
    Drive2.attach(DRIVE2_PIN);
    Fan1.attach(FAN1_PIN);
    Fan2.attach(FAN2_PIN);
    // Stop drive motors initially
    Drive1.writeMicroseconds(STOP);
    Drive2.writeMicroseconds(STOP);

    Fan1.writeMicroseconds(STOP);
    Fan2.writeMicroseconds(STOP);
}

void setDriveSpeed(int speed1, int speed2) {
    int pulseWidth1 = STOP - speed1;
    int pulseWidth2 = STOP + speed2;

    if (pulseWidth1 > FULLFORWARD) pulseWidth1 = FULLFORWARD;
    if (pulseWidth1 < FULLREVERSE) pulseWidth1 = FULLREVERSE;

    if (pulseWidth2 > FULLFORWARD) pulseWidth2 = FULLFORWARD;
    if (pulseWidth2 < FULLREVERSE) pulseWidth2 = FULLREVERSE;

    Drive1.writeMicroseconds(pulseWidth1);
    Drive2.writeMicroseconds(pulseWidth2);

}

void setFanSpeed(int speed) {
    int pulse1 = STOP - speed; //RHS fan
    int pulse2 = STOP - speed; //+ FAN_OFFSET; LHS fan
    Fan1.writeMicroseconds(pulse1);
    Fan2.writeMicroseconds(pulse2);
}

void motorTask() {
    setDriveSpeed(0, 0);
    setFanSpeed(450);
}