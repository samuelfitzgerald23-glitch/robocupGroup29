#include <Arduino.h>
#include "motor.h"
#include <TimerThree.h> // grants access to timer 3 on the board

#define BOARDFREQUENCY 600000000 //600MHz for teensy 4.0

#define MOTORFREQUENCY 50 // 50Hz

#define MOTORPERIOD (BOARDFREQUENCY / MOTORFREQUENCY) // Calculate the period for the motor control signal

bool motorFlag = 0;


//initial commit put line to confirm that the project is working: |

void setup() {
  // put your setup code here, to run once:
  Timer3.initialize(MOTORPERIOD);
  Timer3.attachInterrupt(raiseMotor_flag);
  initMotors();

  Timer3.start();
}

void loop() {
  // put your main code here, to run repeatedly:
  


}

void raiseMotor_flag(void) {
  motorFlag = 1;
}