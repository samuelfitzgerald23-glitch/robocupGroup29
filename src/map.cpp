#include "map.h"
#include <Arduino.h>

/*

TODO: 
constrain x and y?
add location task to task manager

*/

static int mapData[Y_MAX][X_MAX];

static location_sensor_t filtered_sensors[BUFFERCOUNT] = {
    {0, TOF1, 157, 35},
    {0, TOF2, 175, 360-45}
};

void update_map(pos_t* pos, int8_t def) {
    mapData[std::clamp((int)pos->y + Y_ZERO, 0, Y_MAX-1)][std::clamp((int)pos->x + X_ZERO, 0, X_MAX-1)] = def;
}

int16_t getPointDef(pos_t* pos) {
    return mapData[std::clamp((int)pos->y + Y_ZERO, 0, Y_MAX-1)][std::clamp((int)pos->x + X_ZERO, 0, X_MAX-1)];
}

void findWalls() {
    for (int i = 0; i < BUFFERCOUNT; i++) {
        pos_vector_t sensorPOS;
        filtered_sensors[i].SensorOutput = getOutput(filtered_sensors[i].ID);

        calcSensorOutputPos(&sensorPOS, filtered_sensors[i].SensorOutput);
        determineAbsPos(&sensorPOS);

        Serial.print("Distance: ");
        Serial.print(filtered_sensors[i].SensorOutput);

        Serial.print("  theta: ");
        Serial.print(sensorPOS.theta, 3);

        Serial.print("  x: ");
        Serial.print(sensorPOS.pos.x);

        Serial.print("  y: ");
        Serial.println(sensorPOS.pos.y);

        update_map(&(sensorPOS.pos), static_cast<int>(INVALID_CELL));
    }
}

void locationTask() {

    //delete when testing actual navigation
    memset(mapData, 0, sizeof(mapData));
    findWalls();    

    for (int i = 0; i < Y_MAX; i++) {
        for (int j = 0; j < X_MAX; j++) {
            Serial.print(mapData[i][j]);
            Serial.print("\t"); // Use a tab character to separate columns cleanly
        }
        Serial.print("y = "); // Move to the next line after completing a row
        Serial.print(i);
        Serial.println();
    }
}