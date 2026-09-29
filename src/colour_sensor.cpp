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


static MobileType pendingMobileType = NO_MOBILE;

static MobileType classifyMobileElement(void)
{
    bool gold =
        mobileRed >= GOLD_RED_MIN &&
        mobileRed <= GOLD_RED_MAX &&
        mobileBlue >= GOLD_BLUE_MIN &&
        mobileBlue <= GOLD_BLUE_MAX &&
        mobileGreen >= GOLD_GREEN_MIN &&
        mobileGreen <= GOLD_GREEN_MAX &&
        mobileClear >= GOLD_CLEAR_MIN &&
        mobileClear <= GOLD_CLEAR_MAX;

    bool bludger =
        mobileRed >= BLUDGER_TOL_RED_MIN &&
        mobileBlue >= BLUDGER_TOL_BLUE_MIN &&
        mobileGreen >= BLUDGER_TOL_GREEN_MIN &&
        mobileClear >= BLUDGER_TOL_CLEAR_MIN;

    // Prefer gold if readings fall in both ranges.
    if (gold) return GOLD_MOBILE;
    if (bludger) return BLUDGER_MOBILE;

    return NO_MOBILE;
}

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
  //printColourPlot();
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

static void runMobileDetection(MobileMode mode)
{
    MobileType detected = classifyMobileElement();

    // No new detection while a snitch is already held.
    if (mode == CLASSIFY_MOBILE && goodMobileElementPresent) {
        return;
    }

    // Already confirmed: wait until the object leaves before rearming.
    if (mobileElementPresent) {
        if (detected == NO_MOBILE) {
            mobileElementPresent = false;
            pendingMobileType = NO_MOBILE;
        }
        return;
    }

    // A gold snitch is accepted immediately (no confirmation delay).
    if (mode == CLASSIFY_MOBILE && detected == GOLD_MOBILE) {
        mobileCheckPending = false;
        pendingMobileType = NO_MOBILE;

        mobileElementPresent = true;
        goodMobileElementPresent = true;

        Serial.println("Gold snitch detected; holding in centre");
        return;
    }

    // Nothing recognised: cancel any pending confirmation.
    if (detected == NO_MOBILE) {
        if (mobileCheckPending) {
            Serial.println("Mobile confirmation cancelled");
        }

        mobileCheckPending = false;
        pendingMobileType = NO_MOBILE;
        return;
    }

    // Start or restart confirmation if the detected type changes.
    if (!mobileCheckPending || detected != pendingMobileType) {
        mobileCheckPending = true;
        pendingMobileType = detected;
        mobileCheckStartTime = millis();

        if (detected == GOLD_MOBILE) {
            Serial.println("Possible gold snitch; confirming");
        } else {
            Serial.println("Possible bludger; confirming");
        }

        return;
    }

    // Require the same classification for the full confirmation period.
    if (millis() - mobileCheckStartTime < MOBILE_CONFIRM_DELAY) {
        return;
    }

    mobileCheckPending = false;
    mobileElementPresent = true;

    bool keepElement =
        (mode == KEEP_ALL_MOBILE) ||
        (mode == CLASSIFY_MOBILE && detected == GOLD_MOBILE);

    if (keepElement) {
        goodMobileElementPresent = true;

        if (detected == GOLD_MOBILE) {
            Serial.println("Gold snitch confirmed; holding in centre");
        } else {
            Serial.println("Mobile element confirmed; holding in centre");
        }
    } else {
        goodMobileElementPresent = false;

        if (detected == BLUDGER_MOBILE) {
            Serial.println("Bludger confirmed; ejecting");
        } else {
            Serial.println("Gold snitch confirmed; ejecting");
        }

        ejectMobileElement();
    }
}

void plastic_weight_check(void)
{
    bool possiblePlastic =
        (mobileClear >= PLASTIC_WEIGHT_CLEAR_MIN) &&
        (mobileClear <= PLASTIC_WEIGHT_CLEAR_MAX) &&
        (mobileRed >= PLASTIC_WEIGHT_RED_MIN) &&
        (mobileRed <= PLASTIC_WEIGHT_RED_MAX) &&
        (mobileBlue >= PLASTIC_WEIGHT_BLUE_MIN) &&
        (mobileBlue <= PLASTIC_WEIGHT_BLUE_MAX) &&
        (mobileGreen >= PLASTIC_WEIGHT_GREEN_MIN) &&
        (mobileGreen <= PLASTIC_WEIGHT_GREEN_MAX);

    // Check if colour matches plastic and proximity has not detected metal
    if (possiblePlastic &&
        !get_metal_weight_detected() &&
        !plasticWeightPresent) {

        Serial.println("Plastic weight detected!");
        plasticWeightPresent = true;
        sortUnwantedWeight();
    }

    // Rearm once the current weight no longer matches plastic
    if (!possiblePlastic) {
        plasticWeightPresent = false;
    }
}

void mobile_element_check(void)
{
    runMobileDetection(CLASSIFY_MOBILE);
}

void mobile_element_reject_check(void)
{
    runMobileDetection(REJECT_ALL_MOBILE);
}

void mobile_element_keep_check(void)
{
    runMobileDetection(KEEP_ALL_MOBILE);
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