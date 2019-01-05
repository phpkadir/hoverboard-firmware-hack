//BETA V0.1
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "weaking.h"
extern volatile unsigned int current_limit;
extern int currentlr[2];
extern unsigned int timer[2];
extern uint8_t last_pos[2];
extern int phase_period[2];
extern int blockcurlr[2];

extern uint32_t battery_voltage;

extern volatile WeakingPtr currentWeaking;

void stop_buzzer();

unsigned long get_mainCounter();

void set_bldc_motors(bool enable);
void set_throttle(int left,int right);

void set_buzzer(void* buzzerfunc);

void bldc_start_calibration();
void bldc_load_calibration(void* buffer);
