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

//tolerances for snitch/bludger detection (need to update these)
#define MOBILE_TOL_RED 250 
#define MOBILE_TOL_BLUE 120 
#define MOBILE_TOL_GREEN 160

void colour_init(void);
void colourTask(void);
uint16_t get_clear_value(void);
void printColourPlot(void);
void plastic_weight_check(void);
void mobile_element_check(void);

#endif //COLOUR_SENSOR_H