//Author: Samuel Fitzgerald
//Date: 27/5/2026
//Description: This file contains the declarations for the sensor reading functions.

#ifndef TOFS_H
#define TOFS_H

#include <Wire.h>
#include <VL53L1X.h>
#include <SparkFunSX1509.h>

//value written when a TOF has no valid target (bigger than MAX_SIGHT_DIST)
#define TOF_NO_TARGET 8000

void TOFinit(SX1509* io);

void TOF_task();

#endif // TOFS_H