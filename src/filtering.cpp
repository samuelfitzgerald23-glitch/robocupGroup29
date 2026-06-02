#include <stdint.h>
#include "stdlib.h"
#include "circBuf.h"

void filteringInit(void) {
    bufArrayInit();
}

float signalAverage(circBuf_t *buffer) {
    float sum = 0;
    for (uint8_t i = 0; i < buffer->size; i++) {
        sum += readCircBuf(buffer);
    }
    return sum/(buffer->size);
}

uint16_t filterTask(BufID ID) {
    return (uint16_t)signalAverage(getBuf(ID));
}