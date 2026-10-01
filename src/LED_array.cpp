#include "LED_array.h"

CRGB leds[NUM_LEDS];

void LED_init() {
    Serial.println("LED init");
    FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
    FastLED.setBrightness(130);

    for(int i = 0; i < NUM_LEDS; i++) {
        leds[i] = CRGB(255,255,255);
    }
    FastLED.show();
}

void setLEDColor(int index, CRGB color) {
    if (index >= 0 && index < NUM_LEDS) {
        leds[index] = color;
        FastLED.show();
    }
    Serial.println("set led colour");
}