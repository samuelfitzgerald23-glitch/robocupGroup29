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
      Serial.println("Sensor detected");
    } else {
      Serial.print("Failed to detect and initialize sensor ");
      Serial.println(i);
      //while (1);
    }

    // Each sensor must have its address changed to a unique value other than
    // the default of 0x29 (except for the last one, which could be left at
    // the default). To make it simple, we'll just count up from 0x2A.
    sensors[i].setAddress(VL53L1X_ADDRESS_START + i);

    sensors[i].setROISize(16, 4);  //set region of interest (default is 16x16)

    //FIX: init() sets a 50 ms timing budget, but the period was 35 ms.
    //The period must be >= the budget, so set a 33 ms budget to match.
    sensors[i].setMeasurementTimingBudget(33000);
    sensors[i].startContinuous(35);
  }
}

void TOF_task() {
  //FIX: non-blocking. The old readRangeContinuousMillimeters() waited for each
  //sensor's next measurement, stalling the whole scheduler for tens of ms.
  //Now we only read a sensor when it actually has new data.
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    if (!sensors[i].dataReady()) continue;

    uint16_t range = sensors[i].read(false);

    //FIX: check the range status. Invalid readings (no target, signal fail,
    //timeout = 0) used to go straight into the map as an obstacle at 0 mm.
    VL53L1X::RangeStatus status = sensors[i].ranging_data.range_status;
    if (status == VL53L1X::RangeValid && range > 0) {
      TOF_reading[i] = range;
    } else {
      TOF_reading[i] = TOF_NO_TARGET;      //treat as "nothing seen"
    }

    writeCircBuf(getBuf((BufID)i), TOF_reading[i]);
  }
}