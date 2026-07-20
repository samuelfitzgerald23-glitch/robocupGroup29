//Author: Samuel Fitzgerald
//Date: 09/07/2026
//Description: functions for the IMU (BNO055)

#ifndef IMU_H
#define IMU_H

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

void IMU_init();

void printIMUEvent(sensors_event_t* event);

void IMU_task();

float getHeading();

float intergrateIMU(float timestep);

#endif //IMU_H