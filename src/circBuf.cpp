#include <stdint.h>
#include "stdlib.h"
#include <vector>
#include "circBuf.h"

/*
circBuf_t buffer;
bufferInit(x, buffer);
*/

static circBuf_t buffersArray[BUFFERCOUNT];

void bufArrayInit(void) {
    bufferInit(BUFSIZE, &buffersArray[TOF1]);
    bufferInit(BUFSIZE, &buffersArray[TOF2]);
    bufferInit(BUFSIZE, &buffersArray[DIFFTOF1]);
    bufferInit(BUFSIZE, &buffersArray[DIFFTOF2]);
    bufferInit(BUFSIZE, &buffersArray[COLOUR_CLEAR]);
    bufferInit(BUFSIZE, &buffersArray[COLOUR_RED]);
    bufferInit(BUFSIZE, &buffersArray[COLOUR_GREEN]);
    bufferInit(BUFSIZE, &buffersArray[COLOUR_BLUE]);
}

void bufferInit(uint16_t size, circBuf_t *Buffer) {
    Buffer->size = size;
    Buffer->data = new uint16_t[size]();
    Buffer->rindex = 0;
    Buffer->windex = 0;
}

void writeCircBuf(circBuf_t *buffer, uint16_t entry) {
    buffer->data[buffer->windex] = entry;
    buffer->windex++;
    if (buffer->windex >= buffer->size) buffer->windex = 0;
}

uint16_t readCircBuf(circBuf_t *buffer) {
    uint16_t val = buffer->data[buffer->rindex];
    buffer->rindex++;
    if (buffer->rindex >= buffer->size) buffer->rindex = 0;
    return val;
}

void freeCircBuf(circBuf_t *buffer) {
    delete[] buffer->data;
    buffer->data = nullptr;
    buffer->size = 0;
    buffer->rindex=0;
    buffer->windex=0;
}

circBuf_t *getBuf(BufID ID) {
    if(ID>=BUFFERCOUNT) return nullptr;
    
    return &buffersArray[ID];
}
