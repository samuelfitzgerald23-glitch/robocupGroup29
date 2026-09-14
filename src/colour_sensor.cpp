#include <Wire.h>
#include <Arduino.h>
#include "proximity.h"
#include "smart_servo.h"
#include <Adafruit_TCS34725.h>

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_16X);


void colour_init() 
{
  Serial.println("Color View Test!");

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
  
  Serial.print("Clear:\t"); Serial.print(clear);
  //Serial.print("\tRed:\t"); Serial.print(red);
  //Serial.print("\tGreen:\t"); Serial.print(green);
  //Serial.print("\tBlue:\t"); Serial.println(blue);

  if ((clear >= 3000) && !get_metal_weight_detected()) {
      Serial.println("Plastic weight detected!");
      sortUnwantedWeight();
  }

}
