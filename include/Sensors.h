//Author: Samuel Fitzgerald
//Date: 05/07/2026
//Description: functions to organise and streamline sensor initialisation

#ifndef SENSOR_H
#define SENSOR_H

#include "TOFs.h"
#include "IMU.h"

void I2C_scan();

void sensorInit(SX1509* io);

#endif // SENSOR_H