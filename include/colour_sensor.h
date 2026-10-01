#ifndef COLOUR_SENSOR_H
#define COLOUR_SENSOR_H

#include <Wire.h>
#include <Arduino.h>

#include <Adafruit_TCS34725.h>

#include "proximity.h"
#include "smart_servo.h"
#include "circBuf.h"
#include "filtering.h"


//colour output tolerances for a plastic weight
#define PLASTIC_WEIGHT_CLEAR_MIN 400 
#define PLASTIC_WEIGHT_CLEAR_MAX 600
#define PLASTIC_WEIGHT_RED_MIN 200
#define PLASTIC_WEIGHT_RED_MAX 320
#define PLASTIC_WEIGHT_BLUE_MIN 80
#define PLASTIC_WEIGHT_BLUE_MAX 200
#define PLASTIC_WEIGHT_GREEN_MIN 100
#define PLASTIC_WEIGHT_GREEN_MAX 200 

//tolerances for bludger detection (minimums)
#define BLUDGER_TOL_RED_MIN   400
#define BLUDGER_TOL_RED_MAX   1600
#define BLUDGER_TOL_BLUE_MIN  120
#define BLUDGER_TOL_BLUE_MAX  250 
#define BLUDGER_TOL_GREEN_MIN 160
#define BLUDGER_TOL_GREEN_MAX 280 
#define BLUDGER_TOL_CLEAR_MIN 600
#define BLUDGER_TOL_CLEAR_MAX 1600   //(was a 2nd BLUDGER_TOL_GREEN_MAX define - clashed, and is unused)

//tolerances for snitch detection
#define GOLD_RED_MIN    280
#define GOLD_RED_MAX    500
#define GOLD_BLUE_MIN   100
#define GOLD_BLUE_MAX   400
#define GOLD_GREEN_MIN  200
#define GOLD_GREEN_MAX  900
#define GOLD_CLEAR_MIN  600
#define GOLD_CLEAR_MAX  2400

#define MOBILE_STARTUP_DELAY 2000
#define MOBILE_CONFIRM_DELAY 6000 //6s to confirm it is a mobile element (fans are off the whole time)
#define MOBILE_DROPOUT_MS    500  //readings may drop out of the mobile range for this long without cancelling the 5s check
#define MOBILE_SUSPECT_MS    200  //readings must look like a mobile element this long before the robot stops shaking / treats it as one
#define SNITCH_GREEN_MIN     350  //smoothed green above this at any point in the 5s window = SNITCH, otherwise assume BLUDGER

#define MOBILE_AVG_SIZE 20

#define PLASTIC_CONFIRM_DELAY 5000

enum MobileType {
    NO_MOBILE,
    GOLD_MOBILE,
    BLUDGER_MOBILE
};

enum MobileMode {
    CLASSIFY_MOBILE,
    REJECT_ALL_MOBILE,
    KEEP_ALL_MOBILE
};

void colour_init(void);
void colourTask(void);
void printColourPlot(void);
void plastic_weight_check(void);
void mobile_element_check(void);
void mobile_element_reject_check(void);
void mobile_element_keep_check(void);
bool get_bludger_present(void);
bool get_snitch_present(void);
bool get_mobile_suspected(void);   //possible mobile element (early guess), being checked, being ejected or held: reactive.cpp must not shake
uint16_t get_clear_value(void);
void updateMobileAverage(void);
void cancelMobileDetection(void);

#endif //COLOUR_SENSOR_H