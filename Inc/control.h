//ALPHA V0.1
#pragma once
#include <stdint.h>
#include "hd44780.h"

extern LCD_PCF8574_HandleTypeDef lcd;
extern uint32_t timeout;

#ifdef CONTROL_PPM
extern volatile uint16_t ppm_captured_value[PPM_NUM_CHANNELS+1];
#endif

void turnOff();
void turnOffWithReset();

int clean_adc(uint32_t inval);
int calc_torque(int throttle,int breaks);
void calc_torque_per_wheel(int throttle, float steering_eagle, int* torque);
float calc_steering_eagle(int inval);