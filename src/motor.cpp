#include <Servo.h>
#include "motor.h"
#include <Arduino.h>

//driving motors
#define DRIVE1_PIN 2    //left
#define DRIVE2_PIN 3    //right

//fan blade motors
#define FAN1_PIN 5 //RHS
#define FAN2_PIN 4 //LHS
#define FAN_OFFSET 180

//motor speed limits
#define FULLREVERSE 1125
#define STOP 1500
#define FULLFORWARD 1875

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
    delay(10);

    //start fans
    //setFanSpeed(450);
}

void setDriveSpeed(int speed1, int speed2) {
    int pulseWidth1 = constrain((STOP - speed1), FULLREVERSE, FULLFORWARD);
    int pulseWidth2 = constrain((STOP + speed2), FULLREVERSE, FULLFORWARD);
    Serial.print("motor 1, 2 speeds: ");
    Serial.print(pulseWidth1);
    Serial.print(" ");
    Serial.println(pulseWidth2);

    Drive1.writeMicroseconds(pulseWidth1);
    Drive2.writeMicroseconds(pulseWidth2);

}

void setFanSpeed(int speed) {
    int pulse1 = STOP + speed; //RHS fan
    int pulse2 = STOP - speed; //+ FAN_OFFSET; LHS fan
    Fan1.writeMicroseconds(pulse1);
    Fan2.writeMicroseconds(pulse2);
}

void motorTask() {
    Serial.println("motor task");
}