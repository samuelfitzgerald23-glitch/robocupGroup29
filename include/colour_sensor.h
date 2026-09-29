#ifndef COLOUR_SENSOR_H
#define COLOUR_SENSOR_H

#include <Wire.h>
#include <Arduino.h>

#include <Adafruit_TCS34725.h>

#include "proximity.h"
#include "smart_servo.h"
#include "circBuf.h"
#include "filtering.h"

#define PLASTIC_WEIGHT_TOL 1200 //if above this, is plastic. 

//tolerances for bludger detection (minimums)
#define MOBILE_TOL_RED 2000
#define MOBILE_TOL_BLUE 1000 
#define MOBILE_TOL_GREEN 1900
#define MOBILE_TOL_CLEAR 5000

//tolerances for snitch detection
#define GOLD_RED_MIN   150
#define GOLD_RED_MAX   170
#define GOLD_BLUE_MIN   65
#define GOLD_BLUE_MAX  90
#define GOLD_GREEN_MIN 85
#define GOLD_GREEN_MAX  105

#define MOBILE_STARTUP_DELAY 2000
#define MOBILE_CONFIRM_DELAY 3000 //4s to confirm it is a mobile element

#define MOBILE_AVG_SIZE 30

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