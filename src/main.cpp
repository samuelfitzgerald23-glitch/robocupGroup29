#include <Arduino.h>
#include "motor.h"


#define MAXFREQUENCY 10000 //10KHZ for teensy 4.0

#define MOTORFREQUENCY 50 // 50Hz

#define MOTORPERIOD (MAXFREQUENCY / MOTORFREQUENCY) // Calculate the period for the motor control signal

IntervalTimer tickTimer;

volatile uint32_t tickMain = 0;

static uint32_t motorNextRun = MOTORPERIOD;

//initial commit

void setup() {
  // put your setup code here, to run once:
  initMotors();

  tickMain = 0;
  tickTimer.priority(0);
  tickTimer.begin(intervalTick, 1000000/MAXFREQUENCY); //  1/10000 S * 1000000 microsecs per sec,  gives period in microseconds
}

void loop() {
  // put your main code here, to run repeatedly:
  if(tickMain > motorNextRun) {
    motorNextRun += MOTORPERIOD;
    motorTask();
  }

}

void intervalTick() {
  //increment tick based on timer interrupt
  tickMain ++;
}