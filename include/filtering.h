//Author: Samuel Fitzgerald
//Date: 02/6/2026
//Description: This file contains the declarations for filtering sensor inputs.

#ifndef FILTERING_H
#define FILTERING_H

#include "circBuf.h"

void filteringInit(void);

float signalAverage(circBuf_t *buffer);

float signalMedian(circBuf_t *buffer);

void filterTask();

int16_t getOutput(BufID ID);

//number of times filterTask has produced new outputs
uint32_t getFilterUpdateCount();

#endif // FILTERING_H
