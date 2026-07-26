//Author: Samuel Fitzgerald
//Date: 27/5/2026
//Description: This file contains the declarations for the sensor reading functions.

#ifndef SENSOR_H
#define SENSOR_H

#include <Adafruit_BNO055.h>

void TOFinit();
void IMU_init();

void printIMUEvent(sensors_event_t* event);

void TOF();
void IMU_task();

void proximity_init();
void proximity_task();

void sensorTask();

#endif // SENSOR_H