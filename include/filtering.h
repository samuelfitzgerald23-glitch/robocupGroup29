//Author: Samuel Fitzgerald
//Date: 02/6/2026
//Description: This file contains the declarations for filtering sensor inputs.

#ifndef FILTERING_H
#define FILTERING_H

#include "circBuf.h"
#include <stdint.h>

void filteringInit(void);

float signalAverage(circBuf_t *buffer);

void filterTask();

#endif // FILTERING_H