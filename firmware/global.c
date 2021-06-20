#include <stddef.h>
#include <stdint.h>
#include "regulator.h"

float current_temperature = 110.0f;
float prev_volt = 0.0f;

float required_temperature = 110.0f;

struct regulator temperature_regulator = {
	.tim = TIM3,
	.ccr = &(TIM4->CCR1),
	.max = &(TIM4->ARR),
	.k_p = 0.2,
	.k_d = 0.2,
	.k_i = 0,
	.errors = {0, 0, 0},
	.prev = 0,
	.iter = 0,
	.err_sum = 0
};

char cmd_buffer[256] = {0};

float required_adc_val = 1; // for temperature calibration

uint16_t disp_brightness = 100;
uint16_t cooler_speed = 100;

uint8_t app_running = 0;

