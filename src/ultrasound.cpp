#include "ultrasound.h"

volatile uint32_t start_left = 0;
volatile uint32_t start_right = 0;

static size_t ultrasound_sensors[N_ULTRASOUND] = {ULTRASOUND_LEFT_TRIG, ULTRASOUND_RIGHT_TRIG};

static float microsecondsToMillimeters(uint32_t microseconds)
{
  return 10*microseconds / 29 / 2;
}

void leftCallback() {
  float dist = microsecondsToMillimeters((micros() - start_left));
  writeCircBuf(getBuf(ULTRASOUND_LEFT), dist);
}

void rightCallback() {
  float dist = microsecondsToMillimeters((micros() - start_left));
  writeCircBuf(getBuf(ULTRASOUND_RIGHT), dist);
}
 
void Ultrasound_init() 
{  
  pinMode(ULTRASOUND_LEFT_TRIG, OUTPUT);            //Setup ultrasound pins
  attachInterrupt(ULTRASOUND_LEFT_ECHO, leftCallback, RISING);

  pinMode(ULTRASOUND_RIGHT_TRIG, OUTPUT);            //Setup ultrasound pins
  attachInterrupt(ULTRASOUND_RIGHT_ECHO, rightCallback, RISING);
}
 
void ultrasound_start(uint8_t Trigger)
{
 
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(Trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(Trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trigger, LOW);
  if (Trigger == ULTRASOUND_LEFT_TRIG) start_left = micros();
  if (Trigger == ULTRASOUND_RIGHT_TRIG) start_right = micros();
 
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
}

void ultrasound_task() {
  for (int i = 0; i < N_ULTRASOUND; i++) {
    ultrasound_start(ultrasound_sensors[i]);
  }
  Serial.print("Ultrasound "); Serial.print(1); Serial.print(": ");
  Serial.print(getOutput(ULTRASOUND_LEFT));
  Serial.print(" Ultrasound "); Serial.print(2); Serial.print(": ");
  Serial.println(getOutput(ULTRASOUND_RIGHT));
}