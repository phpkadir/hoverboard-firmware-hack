#pragma once
#include "weaking.h"
extern int currentlr[2];
extern int pwmlr[2];
extern uint timer[2];
extern uint8_t last_pos[2];
extern int weaklr[2];
extern uint phase_period[2];
extern int blockcurlr[2];

extern float batteryVoltage;

extern volatile WeakingPtr currentWeaking;