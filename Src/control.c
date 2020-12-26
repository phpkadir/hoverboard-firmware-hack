//ALPHA V0.1
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "stm32f1xx_hal.h"
#include "hd44780.h"
#include "defines.h"
#include "setup.h"
#include "control.h"
#include "config.h"
#include "bldc.h"
#include "weaking.h"
#include "buzzertones.h"
#include "eeprom_manager.h"


LCD_PCF8574_HandleTypeDef lcd;
bool lcd_init_ok = false;
TIM_HandleTypeDef TimHandle;
uint8_t ppm_count = 0;
uint32_t timeout = 100;

//volatile uint16_t ppm_captured_value[PPM_NUM_CHANNELS + 1] = {500, 500};
//volatile uint16_t ppm_captured_value_buffer[PPM_NUM_CHANNELS+1] = {500, 500};
//volatile uint8_t ppm_timeout = 0;

DMA_HandleTypeDef hdma_i2c2_rx;
DMA_HandleTypeDef hdma_i2c2_tx;
bool restart;

uint8_t scan_i2c_next_address(uint8_t start_address){
  return 255;
}

void init_Display(uint8_t lines, uint8_t address){
  
        lcd.pcf8574.PCF_I2C_ADDRESS = address;
        lcd.pcf8574.PCF_I2C_TIMEOUT = 5;
        lcd.pcf8574.i2c = hi2c2;
        switch (lines){
          case 1:
          lcd.NUMBER_OF_LINES = NUMBER_OF_LINES_1;
          break;
          case 2:
          lcd.NUMBER_OF_LINES = NUMBER_OF_LINES_2;
          break;
          case 3:
          lcd.NUMBER_OF_LINES = NUMBER_OF_LINES_3;
          break;
          case 4:
          lcd.NUMBER_OF_LINES = NUMBER_OF_LINES_4;
          break;
        }
        lcd.type = TYPE0;

        if(LCD_Init(&lcd)!=LCD_OK)
            // error occured
            //TODO while(1);
            set_buzzer(noLCD);
        else
            lcd_init_ok = true;

      LCD_ClearDisplay(&lcd);
      HAL_Delay(5);
      LCD_DisplayON(&lcd);
      LCD_SetLocation(&lcd, 0, 0);
      LCD_WriteString(&lcd, "LDEFWH V2.1");
      LCD_SetLocation(&lcd, 0, lines / 2);
      LCD_WriteString(&lcd, "Initializing...");
}

void Display_set_cursor(uint8_t x, uint8_t y){
  LCD_SetLocation(&lcd, x, y);
}

int _Display_show_int(long number){
  LCD_WriteNumber(&lcd, number, 10);
  return log10l(number);
}
int _Display_show_float(float number){
  LCD_WriteFloat(&lcd, number, 3);
  return 3;
}
int _Display_show_string(char* string){
 LCD_WriteString(&lcd, string);
 return strlen(string);
}


int Display_show_int(int8_t x, uint8_t y, long number){
  int tmp = log10l(number);
  if(x<0)
    Display_set_cursor(-(x + tmp), y);
  else
    Display_set_cursor(x, y);
  return _Display_show_int(number);
}

int Display_show_float(int8_t x, uint8_t y, float number, uint8_t len){
  if(x<0)
    Display_set_cursor(-(x+len), y);
  else
    Display_set_cursor(x, y);
  LCD_WriteFloat(&lcd, number, len);
  len;
}
int Display_show_string(int8_t x, uint8_t y, char* string){
  int tmp = strnlen(string, 20);
  if(x<0)
    Display_set_cursor(-(x + tmp), y);
  else
    Display_set_cursor(x, y);
  return _Display_show_string(string);
}

void fallback_defect_latch(){  // if tis code ist executed the board is defect
  stop_buzzer();
  set_bldc_to_led();
  while(1)
    if (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {  // turnoff mechanism
      bool btn_release = false;
      unsigned long startTime = get_mainCounter();
      set_bldc_motors(false);
      while(get_mainCounter() < (startTime + (PWM_FREQ / 5))){  // check button for 0.2s for release to only turn off if its pressed for 2 secs
        if(!HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)){  // if button released
          btn_release = true;
          break;
        }
      }
      if(btn_release){
        set_buzzer(buttonRelease);
        set_bldc_motors(true);
        // do something
      }
      else{
        break;
      }
    }
}

 //BETA V0.1 WORKING
void turnOff(){
  set_bldc_motors(false);
  save_eeprom();
  if(lcd_init_ok)
    LCD_DisplayOFF(&lcd);
  //i2c send turnoff commmand
  restart = false;
  HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, 0);
  fallback_defect_latch();  // never reached
}

 //BETA V0.1 WORKING
void turnOffWithReset(){
  set_bldc_motors(false);
  //i2c send reset+turnoff command
  reset_eeprom();
  if(lcd_init_ok)
    LCD_DisplayOFF(&lcd);
  restart = false;
  HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, 0);
  fallback_defect_latch();  // never reached
}

/*
bool ppm_valid = true;

void PPM_ISR_Callback() {
  // Dummy loop with 16 bit count wrap around
  uint16_t rc_delay = TIM2->CNT;
  TIM2->CNT = 0;

  if (rc_delay > 3000) {
    if (ppm_valid && ppm_count == PPM_NUM_CHANNELS) {
      ppm_timeout = 0;
      memcpy(ppm_captured_value, ppm_captured_value_buffer, sizeof(ppm_captured_value));
    }
    ppm_valid = true;
    ppm_count = 0;
  }
  else if (ppm_count < PPM_NUM_CHANNELS && IN_RANGE(rc_delay, 850, 2150)){
    timeout = 0;
    ppm_captured_value_buffer[ppm_count++] = CLAMP(rc_delay, 1000, 2000) - 1000;
  } else {
    ppm_valid = false;
  }
}

// SysTick executes once each ms
void PPM_SysTick_Callback() {
  ppm_timeout++;
  // Stop after 500 ms without PPM signal
  if(ppm_timeout > 200) {
    int i;
    for(i = 0; i < PPM_NUM_CHANNELS; i++) {
      ppm_captured_value[i] = 500;
    }
    ppm_timeout = 0;
  }
}

void PPM_Init() {
  GPIO_InitTypeDef GPIO_InitStruct;
  // Configure GPIO pin : PA3
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  __HAL_RCC_TIM2_CLK_ENABLE();
  TimHandle.Instance = TIM2;
  TimHandle.Init.Period = UINT16_MAX;
  TimHandle.Init.Prescaler = (SystemCoreClock/DELAY_TIM_FREQUENCY_US)-1;;
  TimHandle.Init.ClockDivision = 0;
  TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  HAL_TIM_Base_Init(&TimHandle);

  // EXTI interrupt init
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
  HAL_TIM_Base_Start(&TimHandle);
}
*//*
void Nunchuck_Init() {
    //-- START -- init WiiNunchuck
  i2cBuffer[0] = 0xF0;
  i2cBuffer[1] = 0x55;

  HAL_I2C_Master_Transmit(&hi2c2,0xA4,(uint8_t*)i2cBuffer, 2, 100);
  HAL_Delay(10);

  i2cBuffer[0] = 0xFB;
  i2cBuffer[1] = 0x00;

  HAL_I2C_Master_Transmit(&hi2c2,0xA4,(uint8_t*)i2cBuffer, 2, 100);
  HAL_Delay(10);
}

void Nunchuck_Read() {
  i2cBuffer[0] = 0x00;
  HAL_I2C_Master_Transmit(&hi2c2,0xA4,(uint8_t*)i2cBuffer, 1, 100);
  HAL_Delay(5);
  if (HAL_I2C_Master_Receive(&hi2c2,0xA4,(uint8_t*)nunchuck_data, 6, 100) == HAL_OK) {
    timeout = 0;
  } else {
    timeout++;
  }

  if (timeout > 3) {
    HAL_Delay(50);
    Nunchuck_Init();
  }

  //setScopeChannel(0, (int)nunchuck_data[0]);
  //setScopeChannel(1, (int)nunchuck_data[1]);
  //setScopeChannel(2, (int)nunchuck_data[5] & 1);
  //setScopeChannel(3, ((int)nunchuck_data[5] >> 1) & 1);
}
*/