#include "ultrasound.h"

volatile uint32_t start_left = 0;
volatile uint32_t start_right = 0;

static int current_sensor = 0;

static size_t ultrasound_sensors[N_ULTRASOUND] = {ULTRASOUND_LEFT_TRIG, ULTRASOUND_RIGHT_TRIG};

static float microsecondsToMillimeters(uint32_t microseconds)
{
  return 10*microseconds / 29 / 2;
}

void leftCallback() {
  if ((bool)digitalReadFast(ULTRASOUND_LEFT_ECHO)) {
    start_left = micros();
  } else {
    float dist = microsecondsToMillimeters((micros() - start_left));
    writeCircBuf(getBuf(ULTRASOUND_LEFT), dist);
  }
}

void rightCallback() {
  if ((bool)digitalReadFast(ULTRASOUND_RIGHT_ECHO)) {
    start_right = micros();
  } else {
    float dist = microsecondsToMillimeters((micros() - start_right));
    writeCircBuf(getBuf(ULTRASOUND_RIGHT), dist);
  }
}
 
void Ultrasound_init() 
{  
  pinMode(ULTRASOUND_LEFT_TRIG, OUTPUT);            //Setup ultrasound pins
  pinMode(ULTRASOUND_LEFT_ECHO, INPUT_PULLDOWN);
  attachInterrupt(ULTRASOUND_LEFT_ECHO, leftCallback, CHANGE);

  pinMode(ULTRASOUND_RIGHT_TRIG, OUTPUT);            //Setup ultrasound pins
  pinMode(ULTRASOUND_RIGHT_ECHO, INPUT_PULLDOWN);
  attachInterrupt(ULTRASOUND_RIGHT_ECHO, rightCallback, CHANGE);
  Serial.println("ultrasound initialised");
}
 
static void ultrasound_start(uint8_t Trigger)
{
 
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(Trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(Trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trigger, LOW);
 
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
}

void ultrasound_task() {
  if (current_sensor >= 2) current_sensor = 0;
  ultrasound_start(ultrasound_sensors[current_sensor]);
  current_sensor++;
}