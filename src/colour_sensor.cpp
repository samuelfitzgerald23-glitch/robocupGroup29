#include <Wire.h>
#include <Arduino.h>
#include "proximity.h"
#include "smart_servo.h"
#include "circBuf.h"
#include "filtering.h"
#include "colour_sensor.h"
#include <Adafruit_TCS34725.h>

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_60X);

// Latch one detection while the same plastic weight remains under the sensor.
// colour_Task() runs repeatedly, so without this latch it continually restarts
// the sorting timer until the weight is removed.
static bool plasticWeightPresent = false;
static bool mobileElementPresent = false;

static uint16_t filteredClear = 0;
static uint16_t filteredRed = 0;
static uint16_t filteredBlue = 0;
static uint16_t filteredGreen = 0;

#define PLASTIC_WEIGHT_TOL 11000

#define MOBILE_TOL_RED 250 
#define MOBILE_TOL_BLUE 120 
#define MOBILE_TOL_GREEN 160 

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

static void updateColourValue()
{
  uint16_t clear, red, green, blue;

  tcs.setInterrupt(false);      // turn on 
  
  tcs.getRawData(&red, &green, &blue, &clear);

  tcs.setInterrupt(true);  // turn off LED

  writeCircBuf(getBuf(COLOUR_CLEAR), clear);
  writeCircBuf(getBuf(COLOUR_RED), red);
  writeCircBuf(getBuf(COLOUR_BLUE), blue);
  writeCircBuf(getBuf(COLOUR_GREEN), green);

  filteredClear = filterTask(COLOUR_CLEAR);
  filteredRed = filterTask(COLOUR_RED);
  filteredBlue = filterTask(COLOUR_BLUE);
  filteredGreen = filterTask(COLOUR_GREEN); //not filtered yet...
}


void colour_Task() 
{

  updateColourValue();
  printColourPlot();
  plastic_weight_check();
  mobile_element_check();

}


void plastic_weight_check(void)
{
  // check if the colour sensor output is within a tolerance & proximity not triggered (not metal)
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

void mobile_element_check(void)
{
  if ((filteredRed < MOBILE_TOL_RED)&&(filteredBlue < MOBILE_TOL_BLUE)&&(filteredGreen < MOBILE_TOL_GREEN)) {
    mobileElementPresent = true;
  } else {
    mobileElementPresent = false;
  }

}

uint16_t get_clear_value()
{
  return filteredClear;
}

void printColourPlot()
{
  Serial.print(">red:");
  Serial.print(filteredRed);

  Serial.print(",green:");
  Serial.print(filteredGreen);

  Serial.print(",blue:");
  Serial.println(filteredBlue);


}