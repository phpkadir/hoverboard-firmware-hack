//FINAL V0.1
#pragma once
#include <stdbool.h>
#include <stdint.h>

void set_buzzerStart(unsigned long mainCnt);

void lowBattery1();
void lowBattery2();
void lowBattery3();

void startUpSound();
void shutDownSound();

void reverseSound();

void buttonRelease();

void resetSound();