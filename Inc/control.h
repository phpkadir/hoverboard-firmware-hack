//ALPHA V0.1
#pragma once
#include <stdint.h>
#include "hd44780.h"
#include "config.h"

extern LCD_PCF8574_HandleTypeDef lcd;
extern uint32_t timeout;
extern bool restart;
extern volatile uint16_t ppm_captured_value[PPM_NUM_CHANNELS+1];

void turnOff();
void turnOffWithReset();

void init_Display(uint8_t lines, uint8_t address);
