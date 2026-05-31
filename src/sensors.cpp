#include <Arduino.h>
#include "sensor.h"

#include <Wire.h>
#include <VL53L0X.h>
#include <SparkFunSX1509.h>

//pins for ultrasound
const int AtrigPin = 3;
const int AechoPin = 2;

const int BtrigPin = 5;
const int BechoPin = 4;

//TOF sensor stuff
const byte SX1509_ADDRESS = 0x3F;
#define VL53L0X_ADDRESS_START 0x30


// The number of sensors in your system.
const uint8_t sensorCount = 2;

// The Arduino pin connected to the XSHUT pin of each sensor.
const uint8_t xshutPins[8] = {0,1,2,3,4,5,6,7};

SX1509 io; // Create an SX1509 object to be used throughout
VL53L0X sensors[sensorCount];

/*
*   TOF sensor functions
*/

static uint16_t TOF_reading[2];

void TOFinit() {
  while (!Serial) {}

  io.begin(SX1509_ADDRESS);

  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  // Disable/reset all sensors by driving their XSHUT pins low.
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    io.pinMode(xshutPins[i], OUTPUT);
    io.digitalWrite(xshutPins[i], LOW);
    //pinMode(xshutPins[i], OUTPUT);
    //digitalWrite(xshutPins[i], LOW);
  }

  // Enable, initialize, and start each sensor, one by one.
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    // Stop driving this sensor's XSHUT low. This should allow the carrier
    // board to pull it high. (We do NOT want to drive XSHUT high since it is
    // not level shifted.) Then wait a bit for the sensor to start up.
    //pinMode(xshutPins[i], INPUT);
    io.digitalWrite(xshutPins[i], HIGH);
    delay(10);

    sensors[i].setTimeout(500);
    if (sensors[i].init())
    {
      Serial.println("Sesnor detected");
    } else {
      Serial.print("Failed to detect and initialize sensor ");
      Serial.println(i);
      while (1);
    }

    // Each sensor must have its address changed to a unique value other than
    // the default of 0x29 (except for the last one, which could be left at
    // the default). To make it simple, we'll just count up from 0x2A.
    sensors[i].setAddress(VL53L0X_ADDRESS_START + i);

    sensors[i].startContinuous(50);
  }
}

void TOF() {
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    TOF_reading[i] = sensors[i].readRangeContinuousMillimeters();

    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(TOF_reading[i]);
    Serial.print(" ");

    if (sensors[i].timeoutOccurred()) { 
      Serial.print(" TIMEOUT"); 
      Serial.print('\t');
    }
  }
  Serial.println();
}

/*
*   main task for sensors
*/

void sensorTask() {
  TOF();
}