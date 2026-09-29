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
#define PLASTIC_WEIGHT_CLEAR_MIN 1100 
#define PLASTIC_WEIGHT_CLEAR_MAX 1300
#define PLASTIC_WEIGHT_RED_MIN 550
#define PLASTIC_WEIGHT_RED_MAX 570
#define PLASTIC_WEIGHT_BLUE_MIN 250
#define PLASTIC_WEIGHT_BLUE_MAX 280
#define PLASTIC_WEIGHT_GREEN_MIN 380
#define PLASTIC_WEIGHT_GREEN_MAX 410 

//tolerances for bludger detection (minimums)
#define BLUDGER_TOL_RED_MIN 2900
#define BLUDGER_TOL_BLUE_MIN 1400
#define BLUDGER_TOL_GREEN_MIN 2200
#define BLUDGER_TOL_CLEAR_MIN 6600

//tolerances for snitch detection
#define GOLD_RED_MIN   2700
#define GOLD_RED_MAX   2900
#define GOLD_BLUE_MIN   1350
#define GOLD_BLUE_MAX  1500
#define GOLD_GREEN_MIN 2200
#define GOLD_GREEN_MAX  2300
#define GOLD_CLEAR_MIN  6400
#define GOLD_CLEAR_MAX  6700

#define MOBILE_STARTUP_DELAY 2000
#define MOBILE_CONFIRM_DELAY 1000 //1s to confirm it is a mobile element

#define MOBILE_AVG_SIZE 20

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
uint16_t get_clear_value(void);
void updateMobileAverage(void);
void cancelMobileDetection(void);

#endif //COLOUR_SENSOR_H