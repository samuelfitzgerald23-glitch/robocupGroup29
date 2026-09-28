#include "colour_sensor.h"

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_60X);

static bool plasticWeightPresent = false;
static bool mobileElementPresent = false;

static uint32_t colourStartTime = 0;
static uint16_t filteredClear = 0;
static uint16_t filteredRed = 0;
static uint16_t filteredBlue = 0;
static uint16_t filteredGreen = 0;

static bool mobileCheckPending = false;
static uint32_t mobileCheckStartTime = 0;

static bool goodMobileElementPresent = false;
static bool goldCandidate = true;


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
  if (millis() - colourStartTime >= MOBILE_STARTUP_DELAY &&
      !isServoSequenceActive()) {
      mobile_element_check();
  }

  // Serial.print("Snitch:");
  // Serial.print(get_snitch_present());

  // Serial.print("bludger:");
  // Serial.print(get_bludger_present());

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
    bool possibleMobile =
        (filteredRed < MOBILE_TOL_RED) &&
        (filteredBlue < MOBILE_TOL_BLUE) &&
        (filteredGreen < MOBILE_TOL_GREEN) && 
        (filteredClear < MOBILE_TOL_CLEAR);

      bool looksGold =
        (filteredRed >= GOLD_RED_MIN) &&
        (filteredRed <= GOLD_RED_MAX) &&
        (filteredBlue >= GOLD_BLUE_MIN) &&
        (filteredBlue <= GOLD_BLUE_MAX) &&
        (filteredGreen >= GOLD_GREEN_MIN) &&
        (filteredGreen <= GOLD_GREEN_MAX);

// Start confirmation when a possible mobile element appears
    if (possibleMobile && !mobileCheckPending &&
        !mobileElementPresent) {

        mobileCheckPending = true;
        mobileCheckStartTime = millis();
        goldCandidate = looksGold;

        Serial.println("Possible mobile element; confirming");
    }

    // During confirmation, remember if readings stop matching gold at any point.
    if (mobileCheckPending) {
        if (!possibleMobile || !looksGold) {
            goldCandidate = false;
        }

        // Wait for the full confirmation period
        if (millis() - mobileCheckStartTime >=
            MOBILE_CONFIRM_DELAY) {

            mobileCheckPending = false;
            mobileElementPresent = true;

            if (possibleMobile && goldCandidate && looksGold) {
                goodMobileElementPresent = true;
                Serial.println("Gold mobile element confirmed");
                // Keep the servo in its centre position
                //setServoPosition(NEUTRAL);
            } else {
                goodMobileElementPresent = false;
                Serial.println("Red/uncertain mobile element; ejecting");
                ejectMobileElement();
            }
        }
    }

    // Rearm once the object has left the sensor
    if (!possibleMobile && !mobileCheckPending) {
        mobileElementPresent = false;
    }
}

bool get_bludger_present(void)
{
    return mobileElementPresent;
}

bool get_snitch_present(void)
{
    return goodMobileElementPresent;
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
  Serial.print(filteredBlue);

  Serial.print(",clear:");
  Serial.println(filteredClear);


}

void cancelMobileDetection(void)
{
    mobileCheckPending = false;
    mobileElementPresent = false;
    goldCandidate = true;
}