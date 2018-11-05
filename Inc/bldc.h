#pragma once
#include "weaking.h"
extern int currentlr[2];
extern int throttlelr[2];
extern uint timer[2];
extern uint8_t last_pos[2];
extern uint phase_period[2];
extern int blockcurlr[2];

extern float batteryVoltage;

extern volatile WeakingPtr currentWeaking;

extern uint8_t buzzerFreq;    // global variable for the buzzer pitch. can be 1, 2, 3, 4, 5, 6, 7...
extern uint8_t buzzerPattern; // global variable for the buzzer pattern. can be 1, 2, 3, 4, 5, 6, 7...

void set_bldc_motors(bool enable);