#ifndef COLOUR_SENSOR_H
#define COLOUR_SENSOR_H

#include <Wire.h>
#include <Arduino.h>

#include <Adafruit_TCS34725.h>

#include "proximity.h"
#include "smart_servo.h"
#include "circBuf.h"
#include "filtering.h"

#define PLASTIC_WEIGHT_TOL 9500

//tolerances for bludger detection 
#define MOBILE_TOL_RED 250
#define MOBILE_TOL_BLUE 120 
#define MOBILE_TOL_GREEN 260
#define MOBILE_TOL_CLEAR 450

//tolerances for snitch detection
#define GOLD_RED_MIN   180
#define GOLD_RED_MAX   230
#define GOLD_BLUE_MIN   85
#define GOLD_BLUE_MAX  130
#define GOLD_GREEN_MIN 110
#define GOLD_GREEN_MAX 170

#define MOBILE_STARTUP_DELAY 2000
#define MOBILE_CONFIRM_DELAY 4000 //4s to confirm it is a mobile element

void colour_init(void);
void colourTask(void);
uint16_t get_clear_value(void);
void printColourPlot(void);
void plastic_weight_check(void);
void mobile_element_check(void);
bool get_bludger_present(void);
bool get_snitch_present(void);
void cancelMobileDetection(void);

#endif //COLOUR_SENSOR_H