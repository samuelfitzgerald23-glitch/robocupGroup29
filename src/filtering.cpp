#include <stdint.h>
#include <Arduino.h>
#include "stdlib.h"
#include "filtering.h"

static int16_t filteredOutputs[BUFFERCOUNT];

//FIX: counts how many times filterTask has run, so the map only uses each
//new filtered reading ONCE (it used to re-use the same value ~20 times)
static volatile uint32_t filterUpdateCount = 0;

int16_t* getFilteredOutputs() {
    return filteredOutputs;
}

void filteringInit(void) {
    bufArrayInit();
}

float signalAverage(circBuf_t *buffer) {
    float sum = 0;
    for (uint16_t i = 0; i < buffer->size; i++) {
        sum += buffer->data[i];
    }
    return sum / buffer->size;
}

//FIX: median rejects single bad samples (0 from a TOF timeout, a missed echo)
//instead of averaging them into the distance
float signalMedian(circBuf_t *buffer) {
    uint16_t tmp[BUFSIZE];
    uint16_t n = (buffer->size < BUFSIZE) ? buffer->size : BUFSIZE;
    for (uint16_t i = 0; i < n; i++) tmp[i] = buffer->data[i];
    //insertion sort (n is tiny)
    for (uint16_t i = 1; i < n; i++) {
        uint16_t v = tmp[i];
        int j = i - 1;
        while (j >= 0 && tmp[j] > v) { tmp[j + 1] = tmp[j]; j--; }
        tmp[j + 1] = v;
    }
    return tmp[n / 2];
}

static bool isDistanceSensor(BufID id) {
    return id == TOF1 || id == TOF2 || id == DIFFTOF1 || id == DIFFTOF2 ||
           id == ULTRASOUND_LEFT || id == ULTRASOUND_RIGHT;
}

void filterTask() {
    for (int i = 0; i < BUFFERCOUNT; i++){
        circBuf_t* buf = getBuf((BufID)i);
        float v = isDistanceSensor((BufID)i) ? signalMedian(buf) : signalAverage(buf);
        filteredOutputs[i] = (int16_t)constrain(roundf(v), -32768.0f, 32767.0f);
    }
    filterUpdateCount++;
    //FIX: removed the per-sensor Serial prints that ran every 100 ms
}

int16_t getOutput(BufID ID) {
    if(ID < 0 || ID>=BUFFERCOUNT) return 0;
    
    return filteredOutputs[ID];
}

uint32_t getFilterUpdateCount() {
    return filterUpdateCount;
}
