#include "map.h"
#include <Arduino.h>

/*

TODO: 
constrain x and y?
add location task to task manager

*/

static uint8_t mapData[Y_MAX][X_MAX];

static location_sensor_t filtered_sensors[BUFFERCOUNT] = {
    {0, TOF1, 1, 30},
    {0, TOF2, 1, -30}
};

void update_map(pos_t* pos, int8_t def) {
    mapData[pos->y + Y_ZERO][pos->x + X_ZERO] = def;
}

int16_t getPointDef(pos_t* pos) {
    return mapData[pos->y][pos->x + X_ZERO];
}

void findWalls() {
    for (int i = 0; i < BUFFERCOUNT; i++) {
        pos_vector_t sensorPOS;
        filtered_sensors[i].SensorOutput = getOutput(filtered_sensors[i].ID);

        calcSensorOutputPos(&sensorPOS, filtered_sensors[i].radialOffset, filtered_sensors[i].angularOffset, filtered_sensors[i].SensorOutput);
        determineAbsPos(&sensorPOS);

        update_map(&(sensorPOS.abs_pos), -1);
    }
}

void locationTask() {

    findWalls();    

    for (int i = 0; i < Y_MAX; i++) {
        for (int j = 0; j < X_MAX; j++) {
            Serial.print(mapData[i][j]);
            Serial.print("\t"); // Use a tab character to separate columns cleanly
        }
        Serial.println(); // Move to the next line after completing a row
    }
}