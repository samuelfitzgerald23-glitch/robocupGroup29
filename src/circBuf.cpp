#include <stdint.h>
#include "stdlib.h"
#include <vector>
#include "circBuf.h"

/*
circBuf_t buffer;
bufferInit(x, buffer);
*/

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

float signalAverage(circBuf_t *buffer) {
    float sum = 0;
    for (uint8_t i = 0; i < buffer->size; i++) {
        sum += readCircBuf(buffer);
    }
    return sum/(buffer->size);
}