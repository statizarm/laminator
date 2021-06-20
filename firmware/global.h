#ifndef GLOBAL_H_
#define GLOBAL_H_

#include "regulator.h"

extern float current_temperature;
extern float prev_volt;

extern float required_temperature;

extern float volt_factor;
extern float volt_to_temp_map[];

extern struct regulator temperature_regulator;
extern char cmd_buffer[];

extern float required_adc_val;
extern uint8_t app_running;

extern uint16_t disp_brightness;
extern uint16_t cooler_speed;


#endif // GLOBAL_H_