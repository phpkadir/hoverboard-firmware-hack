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

extern float batteryVoltage;

extern volatile WeakingPtr currentWeaking;

extern uint8_t buzzerFreq;    // global variable for the buzzer pitch. can be 1, 2, 3, 4, 5, 6, 7...
extern uint8_t buzzerPattern; // global variable for the buzzer pattern. can be 1, 2, 3, 4, 5, 6, 7...

void set_buzzer(bool enable);

void set_bldc_motors(bool enable);
void set_throttle(int left,int right);