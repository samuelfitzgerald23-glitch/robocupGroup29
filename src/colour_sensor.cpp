#include "colour_sensor.h"

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_60X);

static bool plasticWeightPresent = false;
static bool mobileElementPresent = false;

static uint32_t colourStartTime = 0;
static uint16_t filteredClear = 0;
static uint16_t filteredRed = 0;
static uint16_t filteredBlue = 0;
static uint16_t filteredGreen = 0;

void colour_init() 
{
  colourStartTime = millis();

  if (tcs.begin(TCS34725_ADDRESS, &Wire1)) 
  {
    Serial.println("Found colour sensor");
  } else 
  {
    Serial.println("No TCS34725 found ... check your connections");
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

  filteredClear = getOutput(COLOUR_CLEAR);
  filteredRed = getOutput(COLOUR_RED);
  filteredBlue = getOutput(COLOUR_BLUE);
  filteredGreen = getOutput(COLOUR_GREEN);
}


void colourTask() 
{

  updateColourValue();
  //printColourPlot();
  plastic_weight_check();
  
  //don't check for mobile elements until some time passes
  if (millis() - colourStartTime >= MOBILE_STARTUP_DELAY) {
      mobile_element_check();
  }

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
    Serial.println("Mobile element present");
    ejectMobileElement();
  } else {
    mobileElementPresent = false;
  }
}

bool get_mobile_element_present(void)
{
    return mobileElementPresent;
}


uint16_t get_clear_value()
{
  return filteredClear;
}

void printColourPlot()
{
  Serial1.print(">red:");
  Serial.print(filteredRed);

  Serial1.print(",green:");
  Serial.print(filteredGreen);

  Serial1.print(",blue:");
  Serial.print(filteredBlue);

  Serial1.print(",clear:");
  Serial1.println(filteredClear);


}