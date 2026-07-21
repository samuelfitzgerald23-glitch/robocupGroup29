#include <stdint.h>
#include <Arduino.h>
#include "stdlib.h"
#include "circBuf.h"

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
    for (int i = 0; i < BUFFERCOUNT; i++){
        filteredOutputs[i] = (int16_t)roundf(signalAverage(getBuf((BufID)i)));
        Serial.print(filteredOutputs[i]);
        Serial.print(", ");
    }
    Serial.println();
}

int16_t getOutput(BufID ID) {
    if(ID>=BUFFERCOUNT) return 0;
    
    return filteredOutputs[ID];
}

bool determineWeight(BufID ID) {
    //check we wont over index the array
    if ((ID + 2) >= BUFFERCOUNT) return 0;

    int16_t main = filteredOutputs[ID];             //main TOF (on the top of robot)
    int16_t secondary = filteredOutputs[ID + 2];    //differential TOF (middle of robot)

    if (secondary < (main - 100)) return 1;
    return 0;
}