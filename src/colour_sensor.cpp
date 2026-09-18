#include <Wire.h>
#include <Arduino.h>
#include "proximity.h"
#include "smart_servo.h"
#include "circBuf.h"
#include "filtering.h"
#include <Adafruit_TCS34725.h>

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_60X);

// Latch one detection while the same plastic weight remains under the sensor.
// colour_Task() runs repeatedly, so without this latch it continually restarts
// the sorting timer until the weight is removed.
static bool plasticWeightPresent = false;
static uint16_t filteredClear = 0;

#define PLASTIC_WEIGHT_TOL 11000 


void colour_init() 
{
  if (tcs.begin(TCS34725_ADDRESS, &Wire1)) 
  {
    Serial.println("Found colour sensor");
  } else 
  {
    Serial.println("No TCS34725 found ... check your connections");
    //while (1); // halt!
  }
}

static void updateClearValue()
{
  uint16_t clear, red, green, blue;

  tcs.setInterrupt(false);      // turn on 
  
  tcs.getRawData(&red, &green, &blue, &clear);

  tcs.setInterrupt(true);  // turn off LED
  writeCircBuf(getBuf(COLOUR_CLEAR), clear);

  filteredClear = filterTask(COLOUR_CLEAR);
}


void colour_Task() 
{
  updateClearValue();
  
  Serial.print("Clear (filtered):\t"); Serial.println(filteredClear);
  //Serial.print("\tRed:\t"); Serial.print(red);
  //Serial.print("\tGreen:\t"); Serial.print(green);
  //Serial.print("\tBlue:\t"); Serial.println(blue);

  if ((filteredClear >= PLASTIC_WEIGHT_TOL) && !get_metal_weight_detected() && !plasticWeightPresent) {
      Serial.println("Plastic weight detected!");
      plasticWeightPresent = true;
      sortUnwantedWeight();
  }

  // Rearm the detector only after the current weight has left the sensor.
  if (filteredClear < PLASTIC_WEIGHT_TOL) {
      plasticWeightPresent = false;
  }

}

uint16_t get_clear_value()
{
  return filteredClear;
}