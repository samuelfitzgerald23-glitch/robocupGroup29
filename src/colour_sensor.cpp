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


//averaging variables to smooth colour output
static uint16_t mobileRedBuffer[MOBILE_AVG_SIZE] = {0};
static uint16_t mobileGreenBuffer[MOBILE_AVG_SIZE] = {0};
static uint16_t mobileBlueBuffer[MOBILE_AVG_SIZE] = {0};
static uint16_t mobileClearBuffer[MOBILE_AVG_SIZE] = {0};

static uint32_t mobileRedSum = 0;
static uint32_t mobileGreenSum = 0;
static uint32_t mobileBlueSum = 0;
static uint32_t mobileClearSum = 0;

static uint8_t mobileAvgIndex = 0;

static uint16_t mobileRed = 0;
static uint16_t mobileGreen = 0;
static uint16_t mobileBlue = 0;
static uint16_t mobileClear = 0;


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

  updateMobileAverage();
}


void colourTask() 
{
  updateColourValue();
  printColourPlot();
  plastic_weight_check();
  
  //don't check for mobile elements until some time passes
  if (millis() - colourStartTime >= MOBILE_STARTUP_DELAY &&
      !isServoSequenceActive()) {
        
      //do this one if keep snitch, reject bludger  
      //mobile_element_check();
      
      //do this one if want to reject both bludger and snitch
      mobile_element_reject_check();

      //do this if know theres only the snitch in the round - keeps any mobile element inside robot. 
      //mobile_element_keep_check();
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
    bool possibleMobile =
        (mobileRed < MOBILE_TOL_RED) &&
        (mobileBlue < MOBILE_TOL_BLUE) &&
        (mobileGreen < MOBILE_TOL_GREEN) && 
        (mobileClear < MOBILE_TOL_CLEAR);

      bool looksGold =
        (mobileRed >= GOLD_RED_MIN) &&
        (mobileRed <= GOLD_RED_MAX) &&
        (mobileBlue >= GOLD_BLUE_MIN) &&
        (mobileBlue <= GOLD_BLUE_MAX) &&
        (mobileGreen >= GOLD_GREEN_MIN) &&
        (mobileGreen <= GOLD_GREEN_MAX);

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
                // Keep the servo in its centre position - send to home base(so switch to drop_payload state)
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


//only for if reject both bludger AND snitch (colour output not good...)
void mobile_element_reject_check(void)
{
    bool possibleMobile =
        (mobileRed < MOBILE_TOL_RED) &&
        (mobileBlue < MOBILE_TOL_BLUE) &&
        (mobileGreen < MOBILE_TOL_GREEN) && 
        (mobileClear < MOBILE_TOL_CLEAR);

    // Start confirmation
    if (possibleMobile && !mobileCheckPending &&
        !mobileElementPresent) {

        mobileCheckPending = true;
        mobileCheckStartTime = millis();

        Serial.println("Possible mobile element; confirming");
    }

    // Confirm after the delay
    if (mobileCheckPending) {

        // Cancel if the readings no longer indicate an element
        if (!possibleMobile) {
            mobileCheckPending = false;
            Serial.println("Mobile element detection cancelled");
        }

        else if (millis() - mobileCheckStartTime >=
                 MOBILE_CONFIRM_DELAY) {

            mobileCheckPending = false;
            mobileElementPresent = true;
            goodMobileElementPresent = false;

            Serial.println("Mobile element confirmed; ejecting");
            ejectMobileElement();
        }
    }

    // Rearm after the element leaves
    if (!possibleMobile && !mobileCheckPending) {
        mobileElementPresent = false;
    }
}

//if want to keep any mobile element (if know theres only snitch)
void mobile_element_keep_check(void)
{
    bool possibleMobile =
        (mobileRed < MOBILE_TOL_RED) &&
        (mobileBlue < MOBILE_TOL_BLUE) &&
        (mobileGreen < MOBILE_TOL_GREEN) && 
        (mobileClear < MOBILE_TOL_CLEAR);

    // Start confirmation when a possible mobile element appears
    if (possibleMobile && !mobileCheckPending && !mobileElementPresent && !goodMobileElementPresent) {
        mobileCheckPending = true;
        mobileCheckStartTime = millis();
        Serial.println("Possible mobile element; confirming");
    }

    // Confirm after the delay
    if (mobileCheckPending) {

        // Cancel if the object leaves
        if (!possibleMobile) {
            mobileCheckPending = false;
            Serial.println("Mobile element detection cancelled");
        }

        else if (millis() - mobileCheckStartTime >=
                 MOBILE_CONFIRM_DELAY) {

            mobileCheckPending = false;
            mobileElementPresent = true;
            goodMobileElementPresent = true;

            Serial.println("Mobile element detected; holding in centre");

            // Servo should already be at neutral because no servo
            // sequence is active, so leave it there.
        }
    }

    // Rearm once the mobile element leaves
    if (!possibleMobile && !mobileCheckPending) {
        mobileElementPresent = false;
        //goodMobileElementPresent = false;
    }
}

void updateMobileAverage(void)
{
    // Remove old values from the sums
    mobileRedSum -= mobileRedBuffer[mobileAvgIndex];
    mobileGreenSum -= mobileGreenBuffer[mobileAvgIndex];
    mobileBlueSum -= mobileBlueBuffer[mobileAvgIndex];
    mobileClearSum -= mobileClearBuffer[mobileAvgIndex];

    // Add new filtered values
    mobileRedBuffer[mobileAvgIndex] = filteredRed;
    mobileGreenBuffer[mobileAvgIndex] = filteredGreen;
    mobileBlueBuffer[mobileAvgIndex] = filteredBlue;
    mobileClearBuffer[mobileAvgIndex] = filteredClear;

    mobileRedSum += filteredRed;
    mobileGreenSum += filteredGreen;
    mobileBlueSum += filteredBlue;
    mobileClearSum += filteredClear;

    // Move to next buffer position
    mobileAvgIndex++;

    if (mobileAvgIndex >= MOBILE_AVG_SIZE) {
        mobileAvgIndex = 0;
    }

    // Calculate averages
    mobileRed = mobileRedSum / MOBILE_AVG_SIZE;
    mobileGreen = mobileGreenSum / MOBILE_AVG_SIZE;
    mobileBlue = mobileBlueSum / MOBILE_AVG_SIZE;
    mobileClear = mobileClearSum / MOBILE_AVG_SIZE;
}



//for visualizing colour sensor outputs using serial plotter
void printColourPlot()
{
  Serial.print(">red:");
  Serial.print(mobileRed);

  Serial.print(",green:");
  Serial.print(mobileGreen);

  Serial.print(",blue:");
  Serial.print(mobileBlue);

  Serial.print(",clear:");
  Serial.println(mobileClear);

}

void cancelMobileDetection(void)
{
    mobileCheckPending = false;
    mobileElementPresent = false;
    goldCandidate = true;
}

//getter functions
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