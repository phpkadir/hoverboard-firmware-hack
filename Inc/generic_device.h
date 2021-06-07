#pragma once
#include <stdint.h>

#define STEERING_EAGLE_FACTOR (45f/1000f)
#define WHEELBASE 35
#define WHEEL_WIDTH 10
#define STEERING_TO_WHEEL_DIST 5


extern const uint32_t lowBattery_length;
extern const uint32_t lowBattery[];
void device_init();
void device_specific();
void device_button();