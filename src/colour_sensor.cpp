#include <Wire.h>
#include <Arduino.h>
#include "proximity.h"
#include "smart_servo.h"
#include <Adafruit_TCS34725.h>

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_16X);

// Latch one detection while the same plastic weight remains under the sensor.
// colour_Task() runs repeatedly, so without this latch it continually restarts
// the sorting timer until the weight is removed.
static bool plasticWeightPresent = false;


void colour_init() 
{
  if (tcs.begin()) 
  {
    Serial.println("Found colour sensor");
  } else 
  {
    Serial.println("No TCS34725 found ... check your connections");
    //while (1); // halt!
  }
}


void colour_Task() 
{
  uint16_t clear, red, green, blue;

  tcs.setInterrupt(false);      // turn on LED

  delay(60);  // takes 50ms to read 
  
  tcs.getRawData(&red, &green, &blue, &clear);

  tcs.setInterrupt(true);  // turn off LED
  
  //Serial.print("Clear:\t"); Serial.println(clear);
  //Serial.print("\tRed:\t"); Serial.print(red);
  //Serial.print("\tGreen:\t"); Serial.print(green);
  //Serial.print("\tBlue:\t"); Serial.println(blue);

  if ((clear >= 3000) && !get_metal_weight_detected() && !plasticWeightPresent) {
      Serial.println("Plastic weight detected!");
      plasticWeightPresent = true;
      sortUnwantedWeight();
  }

  // Rearm the detector only after the current weight has left the sensor.
  if (clear < 3000) {
      plasticWeightPresent = false;
  }

}


uint16_t get_clear_value()
{
  uint16_t clear, red, green, blue;
  tcs.getRawData(&red, &green, &blue, &clear);
  return clear;
}