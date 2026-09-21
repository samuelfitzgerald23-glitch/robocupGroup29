#ifndef COLOUR_SENSOR_H
#define COLOUR_SENSOR_H


void colour_init(void);
void colourTask(void);
static void updateColourValue(void);
uint16_t get_clear_value(void);
void printColourPlot(void);
void plastic_weight_check(void);
void mobile_element_check(void);

#endif //COLOUR_SENSOR_H