//Author: Samuel Fitzgerald
//Date: 27/5/2026
//Description: This file contains the declarations for the sensor reading functions.

#ifndef TOFS_H
#define TOFS_H

#include <Wire.h>
#include <VL53L1X.h>
#include <SparkFunSX1509.h>

void TOFinit(SX1509* io);

void TOF_task();

#endif // TOFS_H