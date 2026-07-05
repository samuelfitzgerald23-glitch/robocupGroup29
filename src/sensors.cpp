#include "Sensors.h"

void sensorInit(SX1509* io) {
    byte error, add;
    Serial.println("Scanning for I2C devices");
    for(add = 1; add < 127; add++) {
        Wire.beginTransmission(add);
        error = Wire.endTransmission();

        if (error == 0) {
            Serial.print("Found at 0x");
            if(add < 16) Serial.print("0");
            Serial.println(add, HEX);
        }
    }


    TOFinit(io);
    IRCamera_init();
}

void sensorTask() {
    TOF_task();
    IRCamera_task();
}