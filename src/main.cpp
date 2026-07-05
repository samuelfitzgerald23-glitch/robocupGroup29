#include <Arduino.h>
#include "motor.h"
#include "Sensors.h"
#include "filtering.h"
#include <Wire.h>
#include <VL53L0X.h>
#include <SparkFunSX1509.h>


#define MAXFREQUENCY 1000 //1KHZ for teensy 4.0

#define MOTORFREQUENCY 37 // 37Hz
#define SENSORFREQUENCY 50 //50Hz
#define FILTERFREQUENCY 500 //500Hz

#define MOTORPERIOD (MAXFREQUENCY / MOTORFREQUENCY) // Calculate the period for the motor control signal
#define SENSORPERIOD (MAXFREQUENCY / SENSORFREQUENCY)
#define FILTERPERIOD (MAXFREQUENCY /  FILTERFREQUENCY)

const byte SX1509_ADDRESS = 0x3F;
SX1509 io; // Create an SX1509 object to be used throughout

IntervalTimer tickTimer;

volatile uint32_t tickMain = 0;

void intervalTick() {
  //increment tick based on timer interrupt
  tickMain ++;
}

static uint32_t motorNextRun = MOTORPERIOD;
static uint32_t sensorNextRun = SENSORPERIOD;
static uint32_t filterNextRun = FILTERPERIOD;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  io.begin(SX1509_ADDRESS);

  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  initMotors();
  sensorInit(&io);
  filteringInit();

  tickMain = 0;
  tickTimer.priority(0);
  tickTimer.begin(intervalTick, 1000000/MAXFREQUENCY); //  1/1000 S * 1000000 microsecs per sec,  gives period in microseconds
}

void loop() {
  // put your main code here, to run repeatedly:
  noInterrupts();
  uint32_t tickCopy = tickMain;
  interrupts();


  if(tickCopy > motorNextRun) {
    motorNextRun += MOTORPERIOD;
    motorTask();
  }
  
  if(tickCopy > sensorNextRun) {
    sensorNextRun += SENSORPERIOD;
    sensorTask();
  }
  if(tickCopy > filterNextRun) {
    uint16_t TOF1Output = filterTask(TOF1);
    uint16_t TOF2Output = filterTask(TOF2);
    Serial.print(TOF1Output);
    Serial.print(" : ");
    Serial.println(TOF2Output);
    filterNextRun += FILTERPERIOD;
  }
  
}