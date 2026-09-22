#include "xy_optical.h"
#include "FastLED.h"

//if height of sensor is changed this value will need to be re-calibrated manually
//to calibrate record the number of counts over a known distence (e.g 300mm) and find counts/dist

//at ~12cm off the ground this is 1259 counts / 300 mm
#define COUNTS_PER_MM 3.0f

#define NUM_LEDS 14
#define DATA_PIN 33

CRGB leds[NUM_LEDS];

Bitcraze_PMW3901 flow(XY_CS);

void init_optical_flow()
{
    FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
    FastLED.setBrightness(130);

    for(int i = 0; i < NUM_LEDS; i++) {
        leds[i] = CRGB(255,255,255);
    }
    FastLED.show();

    if (!flow.begin()) {
        Serial.println("Initialization of the flow sensor failed");
        while(1) { }
    }
}


//this needs to be changed is the sensor is rotated
void read_opticalFlow(float& deltaX, float& deltaY) {
    //optain raw delta x and y from sensor
    int16_t x, y;
    flow.readMotionCount(&x, &y);

    deltaX = static_cast<float>(x)/COUNTS_PER_MM;
    deltaY = static_cast<float>(y)/COUNTS_PER_MM;
    //FastLED.show();
}