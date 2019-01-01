#pragma once
#include <stdbool.h>
#include <stdint.h>

void set_buzzerStart(unsigned long mainCnt);

void lowBattery1();
void lowBattery2();
void lowBattery3();

void startUpSound();
void shutDownSound();

void buttonRelease();