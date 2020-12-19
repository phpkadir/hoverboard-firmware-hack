//FINAL V0.1
#pragma once
#include <stdbool.h>
#include <stdint.h>

extern const void* lowBatTones[];

void set_buzzerStart(unsigned long mainCnt);

void noLCD();
void noSlave();

void lowBattery1();
void lowBattery2();
void lowBattery3();

void startUpSound();
void shutDownSound();

void reverseSound();

void buttonRelease();

void resetSound();