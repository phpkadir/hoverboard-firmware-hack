#pragma once
#include <stdint.h>

extern uint32_t timeout;
int clean_adc(int inval);
int calc_torque(int throttle,int breaks);
void calc_torque_per_wheel(int throttle, float steering_eagle, int* torque);
float calc_steering_eagle(int inval);