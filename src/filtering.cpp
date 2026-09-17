#include <stdint.h>
#include <Arduino.h>
#include "stdlib.h"
#include "filtering.h"

static int16_t filteredOutputs[BUFFERCOUNT];

int16_t* getFilteredOutputs() {
    return filteredOutputs;
}

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

void filterTask() {
    Serial.println("filter task");
    for (int i = 0; i < BUFFERCOUNT; i++){
        filteredOutputs[i] = (int16_t)roundf(signalAverage(getBuf((BufID)i)));
        Serial.print(BufID(i));
        Serial.print(": ");
        Serial.print(filteredOutputs[i]);
        Serial.print(", ");
    }
    Serial.println();
}

int16_t getOutput(BufID ID) {
    if(ID>=BUFFERCOUNT) return 0;
    
    return filteredOutputs[ID];
}