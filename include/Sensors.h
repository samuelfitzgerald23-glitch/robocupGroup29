//Author: Samuel Fitzgerald
//Date: 05/07/2026
//Description: 

#ifndef SENSOR_H
#define SENSOR_H

#include "TOFs.h"
#include "IRCamera.h"

void I2C_scan();

void sensorInit(SX1509* io);

void sensorTask();

#endif // SENSOR_H