#include <Arduino.h>
#include "motor.h"
#include "sensor.h"


#define MAXFREQUENCY 10000 //10KHZ for teensy 4.0

#define MOTORFREQUENCY 50 // 50Hz
#define SENSORFREQUENCY 2000 //2000Hz

#define MOTORPERIOD (MAXFREQUENCY / MOTORFREQUENCY) // Calculate the period for the motor control signal
#define SENSORPERIOD (MAXFREQUENCY / SENSORFREQUENCY)

IntervalTimer tickTimer;

volatile uint32_t tickMain = 0;

void intervalTick() {
  //increment tick based on timer interrupt
  tickMain ++;
}

static uint32_t motorNextRun = MOTORPERIOD;
static uint32_t sensorNextRun = SENSORPERIOD;

//initial commit

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  initMotors();
  sensorInit();

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
  if(tickMain > sensorNextRun) {
    sensorNextRun += SENSORPERIOD;
    sensorTask();
  }

}