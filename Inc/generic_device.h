#pragma once
#include <stdint.h>

#define STEERING_EAGLE_FACTOR 0.01
#define WHEELBASE 2
#define WHEEL_WIDTH 1
#define STEERING_TO_WHEEL_DIST 1


extern const uint32_t lowBattery_length;
extern const uint32_t lowBattery[];
void device_init();
void device_specific();
void device_button();