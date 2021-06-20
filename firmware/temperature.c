#include "temperature.h"

#define MIN_ADC_VAL (300.0f)


static float __temp_tab[8] = {
	200, 175, 150, 125, 100, 75, 50, 25
};

static int __calib_index = 0xFFFF;

static float __temp_k = 0.01412;

float get_temperature_by_adc_value(float adc_val)
{
	if ((adc_val -= MIN_ADC_VAL) < 0) {
		return MAX_TEMPERATURE;
	}
	
	uint32_t i = adc_val * __temp_k;
	
	if (i > sizeof(__temp_tab) / sizeof(__temp_tab[0]) - 1) {
		return __temp_tab[sizeof(__temp_tab) / sizeof(__temp_tab[0]) - 1];
	}
	
	float frac = 1 - (float) adc_val * __temp_k + (float) i;
	float lowest_ge = __temp_tab[i];
	float greatest_le = __temp_tab[i + 1];
	return greatest_le + (lowest_ge - greatest_le) * frac;
}

void init_temperature_calibration()
{
	extern float required_adc_val;
	__calib_index = sizeof(__temp_tab) / sizeof(__temp_tab[0]) - 1;
	required_adc_val = MIN_ADC_VAL + (float) __calib_index / __temp_k;
}

int push_calibration_value(float temp)
{
	extern float required_adc_val;
	if (__calib_index > sizeof(__temp_tab) / sizeof(__temp_tab[0])) {
		return CAL_NOT_INITIALIZED;
	}
	
	__temp_tab[__calib_index] = temp;
	
	if (temp >= MAX_TEMPERATURE) {
		__calib_index = sizeof(__temp_tab) / sizeof(__temp_tab[0]);
		return CAL_FINISHED;
	}
	
	if (__calib_index == sizeof(__temp_tab) / sizeof(__temp_tab[0])) {
		return CAL_FINISHED;
	}
	
	required_adc_val = MIN_ADC_VAL + (float) (--__calib_index) / __temp_k;
	return CAL_WAITING_NEXT;
}
