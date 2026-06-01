//Author: Samuel Fitzgerald
//Date: 31/5/2026
//Description: This file contains the declarations for the sensor reading functions.

#ifndef CIRCBUF_H
#define CIRCBUF_H

struct circBuf_t {
    uint16_t size;
    uint16_t windex;
    uint16_t rindex;
    uint16_t* data;
};

//given a pointer to a circular buffer, initialise it
void bufferInit(uint16_t size, circBuf_t *Buffer);

//write to the circular buffer
void writeCircBuf(circBuf_t *buffer, uint16_t entry);

//read a value from the circular buffer
uint16_t readCircBuf(circBuf_t *buffer);

//free the memory alocated to the circular buffer
void freeCircBuf(circBuf_t *buffer);

//average the value inside the buffer
uint16_t signalAverage(circBuf_t *buffer);

#endif // CIRCBUF_H