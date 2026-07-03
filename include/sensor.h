//Author: Samuel Fitzgerald
//Date: 27/5/2026
//Description: This file contains the declarations for the sensor reading functions.

#ifndef SENSOR_H
#define SENSOR_H

void TOFinit(SX1509* io);

void TOF();

void sensorTask();

#endif // SENSOR_H