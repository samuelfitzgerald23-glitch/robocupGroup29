#ifndef LED_ARRAY_H
#define LED_ARRAY_H

#include "FastLED.h"

#define NUM_LEDS 14
#define DATA_PIN 33

#define WRITIBLE_LED 6

#define LED_BRIGHTNESS 130

#define WHITE CRGB(255, 255, 255)
#define RED CRGB(255, 0, 0)
#define GREEN CRGB(0, 255, 0)
#define BLUE CRGB(0, 0, 255)

void LED_init();
void setLEDColor(int index, CRGB color);

#endif // LED_ARRAY_H