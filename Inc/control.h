//ALPHA V0.1
#pragma once
#include <stdint.h>
#include "config.h"

extern uint32_t timeout;
extern bool restart;
extern volatile uint16_t ppm_captured_value[PPM_NUM_CHANNELS+1];

void turnOff();
void turnOffWithReset();

void init_Display(uint8_t lines, uint8_t address);
void Display_set_cursor(uint8_t x, uint8_t y);
int _Display_show_int(long number);
int _Display_show_float(float number);
int _Display_show_string(char* string);
int Display_show_int(int8_t x, uint8_t y, long number);
int Display_show_float(int8_t x, uint8_t y, float number, uint8_t len);
int Display_show_string(int8_t x, uint8_t y, char* string);

