#include <Arduino.h>
#include "Sensors.h"

#include <Wire.h>
#include <SparkFunSX1509.h>


/*
known I2C addresses:
IMU: 0x28
IO board: 0x30
TOFs: 
*/

static void I2C_scan(TwoWire buss) {
    //Only scans I2C 0, if want to try other I2C bus replace Wire with Wire1 (I2C 1)...
    //output of address is in 7 bits (typical I2C addressing), Wire wants 8 bits?
    byte error, add;
    Serial.println("Scanning for I2C devices");
    for(add = 1; add < 127; add++) {
        buss.beginTransmission(add);
        error = buss.endTransmission();

        if (error == 0) {
            Serial.print("Found at 0x");
            if(add < 16) Serial.print("0");
            Serial.println(add, HEX);
        }
    }
}

void sensorInit(SX1509* io) {

    //Do I2C scan if cannot find address of I2C device
    I2C_scan(Wire);

    Ultrasound_init();
    TOFinit(io);
    IMU_init();
    init_optical_flow();
    colour_init();
    proximity_init();
}
