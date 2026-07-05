#include "Sensors.h"

void sensorInit(SX1509* io) {

    //Do I2C scan if cannot find address of I2C device
    I2C_scan();

    TOFinit(io);
    IRCamera_init();
}

void sensorTask() {
    TOF_task();
    IRCamera_task();
}

void I2C_scan() {
    //output of address is in 7 bits (typical I2C addressing), Wire wants 8 bits?
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
}