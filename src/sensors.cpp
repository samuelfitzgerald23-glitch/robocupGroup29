#include <Arduino.h>
#include "sensor.h"

//pins for ultrasound
const int AtrigPin = 3;
const int AechoPin = 2;

const int BtrigPin = 5;
const int BechoPin = 4;

void sensorInit() {
    pinMode(AtrigPin, OUTPUT);            //Setup ultrasound pins
    pinMode(AechoPin, INPUT);

    pinMode(BtrigPin, OUTPUT);            //Setup ultrasound pins
    pinMode(BechoPin, INPUT);

    Serial.printf("Sensors on");
}

/*
*   Functions for ultrasound sensors
*/

void ultrasound()
{
  long durationA,durationB, Acm,Bcm;
 
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(AtrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(AtrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(AtrigPin, LOW);
 
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  durationA = pulseIn(AechoPin, HIGH);

  digitalWrite(BtrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(BtrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(BtrigPin, LOW);
  durationB = pulseIn(BechoPin, HIGH);
  
 
  // convert the time into a distance
  Acm = microsecondsToCentimeters(durationA);
  Bcm = microsecondsToCentimeters(durationB);
  Serial.print(Acm);
  Serial.print(" ");
  Serial.print(Bcm);
  Serial.println();
  delay(1000);
}

long microsecondsToCentimeters(long microseconds)
{
  return (microseconds / 29) / 2;
} 

/*
*   main task for sensors
*/

void sensorTask() {
    ultrasound();
}