//Author: Samuel Fitzgerald
//Date: 27/5/2026
//Description: This file contains the declarations for the sensor reading functions.

#ifndef SENSOR_H
#define SENSOR_H

#include <Wire.h>
#include <VL53L0X.h>
#include <SparkFunSX1509.h>

void TOFinit(SX1509* io);

void TOF();

void sensorTask();

#endif // SENSOR_H