#include "Sensors.h"

void sensorInit(SX1509* io) {
    TOFinit(io);
    IRCamera_init();
}

void sensorTask() {
    TOF_task();
    IRCamera_task();
}