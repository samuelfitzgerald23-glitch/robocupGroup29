#include <Arduino.h>
#include "TOFs.h"
#include "circBuf.h"

//TOF sensor stuff
#define VL53L1X_ADDRESS_START 0x30


// The number of sensors in your system.
const uint8_t sensorCount = 4;

// The Arduino pin connected to the XSHUT pin of each sensor.
const uint8_t xshutPins[8] = {0,1,2,3,4,5,6,7};


VL53L1X sensors[sensorCount];

/*
*   TOF sensor functions
*/

static uint16_t TOF_reading[sensorCount];

void TOFinit(SX1509* io) {
  //while (!Serial) {}

  // Disable/reset all sensors by driving their XSHUT pins low.
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    io->pinMode(xshutPins[i], OUTPUT);
    io->digitalWrite(xshutPins[i], LOW);
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
    io->digitalWrite(xshutPins[i], HIGH);
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
    sensors[i].setAddress(VL53L1X_ADDRESS_START + i);

    sensors[i].setROISize(4, 4);  //set region of interest (default is 16x16)

    sensors[i].startContinuous(35);
  }
}

void TOF_task() {
  Serial.println("TOF task");
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    TOF_reading[i] = sensors[i].readRangeContinuousMillimeters();

    // Serial.print("Sensor ");
    // Serial.print(i);
    // Serial.print(": ");
    // Serial.print(TOF_reading[i]);
    // Serial.print(" ");

    if (sensors[i].timeoutOccurred()) { 
      Serial.print(" TIMEOUT"); 
      Serial.print('\t');
    }
    writeCircBuf(getBuf((BufID)i), TOF_reading[i]);
  }
  Serial.println();
}