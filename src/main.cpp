#include <Arduino.h>
#include "motor.h"

#define BOARDFREQUENCY 600000000 //600MHz for teensy 4.0

#define MOTORFREQUENCY 50 // 50Hz

#define MOTORPERIOD (BOARDFREQUENCY / MOTORFREQUENCY) // Calculate the period for the motor control signal

//initial commit put line to confirm that the project is working: |

void setup() {
  // put your setup code here, to run once:
  initMotors();
}

void loop() {
  // put your main code here, to run repeatedly:
}

