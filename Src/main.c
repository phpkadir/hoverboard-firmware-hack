/*
* This file is part of the hoverboard-firmware-hack project.
*
* Copyright (C) 2017-2018 Rene Hopf <renehopf@mac.com>
* Copyright (C) 2017-2018 Nico Stute <crinq@crinq.de>
* Copyright (C) 2017-2018 Niklas Fauth <niklas.fauth@kit.fail>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "stm32f1xx_hal.h"  // main code for stm32 controller
#include "defines.h"  // for the macros
#include "setup.h"  // for access the functions form setup
#include "bldc.h"  // for the main control variables
//#include "hd44780.h"  // for the display
#include "config.h"  // the config
#include "control.h"

void SystemClock_Config(void);

//LCD_PCF8574_HandleTypeDef lcd;

int cmd1;  // normalized input values. -1000 to 1000
int cmd2;
int cmd3;

float adc1_filtered = 0,adc2_filtered = 0;

typedef struct{
   int16_t steer;
   int16_t speed;
   //uint32_t crc;
} Serialcommand;

volatile Serialcommand command;

uint8_t button1, button2;

int steer; // global variable for steering. -1000 to 1000
int speed; // global variable for speed. -1000 to 1000

int mode =2;
extern uint8_t nunchuck_data[6];
#ifdef CONTROL_PPM
extern volatile uint16_t ppm_captured_value[PPM_NUM_CHANNELS+1];
#endif

int milli_vel_error_sum = 0;

int main(void) {
  HAL_Init();
  __HAL_RCC_AFIO_CLK_ENABLE();
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  /* System interrupt init*/
  /* MemoryManagement_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
  /* BusFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
  /* UsageFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
  /* SVCall_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
  /* DebugMonitor_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
  /* PendSV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  SystemClock_Config();

  __HAL_RCC_DMA1_CLK_DISABLE();
  MX_GPIO_Init();
  MX_TIM_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();

  #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
    UART_Init();
  #endif

  HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, 1);

  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc2);

  for (int i = 8; i >= 0; i--) {
    buzzerFreq = i;
    HAL_Delay(100);
  }
  buzzerFreq = 0;

  HAL_GPIO_WritePin(LED_PORT, LED_PIN, 1);

  int lastSpeedL = 0, lastSpeedR = 0;
  int speedL = 0, speedR = 0, speedRL = 0;
  float direction = 1;

  #ifdef CONTROL_PPM
    PPM_Init();
  #endif

  #ifdef CONTROL_NUNCHUCK
    I2C_Init();
    Nunchuck_Init();
  #endif

  #ifdef CONTROL_SERIAL_USART2
    UART_Control_Init();
    HAL_UART_Receive_DMA(&huart2, (uint8_t *)&command, 4);
  #endif

  #ifdef DEBUG_I2C_LCD
    I2C_Init();
    HAL_Delay(50);
    lcd.pcf8574.PCF_I2C_ADDRESS = 0x27;
      lcd.pcf8574.PCF_I2C_TIMEOUT = 5;
      lcd.pcf8574.i2c = hi2c2;
      lcd.NUMBER_OF_LINES = NUMBER_OF_LINES_2;
      lcd.type = TYPE0;

      if(LCD_Init(&lcd)!=LCD_OK){
          // error occured
          //TODO while(1);
      }

    LCD_ClearDisplay(&lcd);
    HAL_Delay(5);
    LCD_SetLocation(&lcd, 0, 0);
    LCD_WriteString(&lcd, "Hover V2.0");
    LCD_SetLocation(&lcd, 0, 1);
    LCD_WriteString(&lcd, "Initializing...");
  #endif

  set_bldc_motors(true);

  while(1) {
    HAL_Delay(5);

    #ifdef CONTROL_ADC
    // ####### larsm's bobby car code #######

    // LOW-PASS FILTER (fliessender Mittelwert)
    adc1_filtered = adc1_filtered * 0.9 + (float)adc_buffer.l_rx2 * 0.1; // links, rueckwearts
    adc2_filtered = adc2_filtered * 0.9 + (float)adc_buffer.l_tx2 * 0.1; // rechts, vorwaerts

    // magic numbers die ich nicht mehr nachvollziehen kann, faehrt sich aber gut ;-)
    #define LOSLASS_BREMS_ACC 0.996f  // naeher an 1 = gemaechlicher
    #define DRUECK_ACC1 (1.0f - LOSLASS_BREMS_ACC + 0.001f)  // naeher an 0 = gemaechlicher
    #define DRUECK_ACC2 (1.0f - LOSLASS_BREMS_ACC + 0.001f)  // naeher an 0 = gemaechlicher
    //die + 0.001f gleichen float ungenauigkeiten aus.

    #define ADC1_DELTA (ADC1_MAX - ADC1_MIN)
    #define ADC2_DELTA (ADC2_MAX - ADC2_MIN)

    if (mode == 1) {  // Mode 1, links: 3 kmh
      speedRL = (float)speedRL * LOSLASS_BREMS_ACC  // bremsen wenn kein poti gedrueckt
              - (CLAMP(adc_buffer.l_rx2 - ADC1_MIN, 0, ADC1_DELTA) / (ADC1_DELTA / 280.0f)) * DRUECK_ACC1  // links gedrueckt = zusatzbremsen oder rueckwaertsfahren
              + (CLAMP(adc_buffer.l_tx2 - ADC2_MIN, 0, ADC2_DELTA) / (ADC2_DELTA / 350.0f)) * DRUECK_ACC2;  // vorwaerts gedrueckt = beschleunigen 12s: 350=3kmh

    } else if (mode == 2) { // Mode 2, default: 6 kmh
      speedRL = (float)speedRL * LOSLASS_BREMS_ACC
              - (CLAMP(adc_buffer.l_rx2 - ADC1_MIN, 0, ADC1_DELTA) / (ADC1_DELTA / 310.0f)) * DRUECK_ACC1
              + (CLAMP(adc_buffer.l_tx2 - ADC2_MIN, 0, ADC2_DELTA) / (ADC2_DELTA / 420.0f)) * DRUECK_ACC2;  // 12s: 400=5-6kmh 450=7kmh

    } else if (mode == 3) { // Mode 3, rechts: 12 kmh
      speedRL = (float)speedRL * LOSLASS_BREMS_ACC
              - (CLAMP(adc_buffer.l_rx2 - ADC1_MIN, 0, ADC1_DELTA) / (ADC1_DELTA / 340.0f)) * DRUECK_ACC1
              + (CLAMP(adc_buffer.l_tx2 - ADC2_MIN, 0, ADC2_DELTA) / (ADC2_DELTA / 600.0f)) * DRUECK_ACC2;  // 12s: 600=12kmh

    } else if (mode == 4) { // Mode 4, l + r: full kmh
      // Feldschwaechung wird nur aktiviert wenn man schon sehr schnell ist. So gehts: Rechts voll druecken und warten bis man schnell ist, dann zusaetzlich links schnell voll druecken.
      if (adc1_filtered > (ADC1_MAX - 450) && speedRL > 800) { // field weakening at high speeds
        speedRL = (float)speedRL * LOSLASS_BREMS_ACC
              + (CLAMP(adc_buffer.l_tx2 - ADC2_MIN, 0, ADC2_DELTA) / (ADC2_DELTA / 1000.0f)) * DRUECK_ACC2;
        weak = weak * 0.95 + 400.0 * 0.05;  // sanftes hinzuschalten des turbos, 12s: 400=29kmh
      } else { //normale fahrt ohne feldschwaechung
        speedRL = (float)speedRL * LOSLASS_BREMS_ACC
              - (CLAMP(adc_buffer.l_rx2 - ADC1_MIN, 0, ADC1_DELTA) / (ADC1_DELTA / 340.0f)) * DRUECK_ACC1
              + (CLAMP(adc_buffer.l_tx2 - ADC2_MIN, 0, ADC2_DELTA) / (ADC2_DELTA / 1000.0f)) * DRUECK_ACC2;  // 12s: 1000=22kmh
        weak = weak * 0.95;  // sanftes abschalten des turbos
      }
    }

    speed = CLAMP(speedRL, -1000, 1000);  // clamp output

      timeout = 0;
    #endif

    // ####### LOW-PASS FILTER #######
    steer = steer * (1.0 - FILTER) + cmd1 * FILTER;
    speed = speed * (1.0 - FILTER) + cmd2 * FILTER;


    // ####### MIXER #######
    speedR = CLAMP(speed * SPEED_COEFFICIENT -  steer * STEER_COEFFICIENT, -1000, 1000);
    speedL = CLAMP(speed * SPEED_COEFFICIENT +  steer * STEER_COEFFICIENT, -1000, 1000);


    // ####### DEBUG SERIAL OUT #######
    #ifdef CONTROL_ADC
      setScopeChannel(0, (int)adc_buffer.l_tx2);  // ADC1
      setScopeChannel(1, (int)adc_buffer.l_rx2);  // ADC2
    #endif
    setScopeChannel(2, (int)speedR);
    setScopeChannel(3, (int)speedL);

    #ifdef ADDITIONAL_CODE
      ADDITIONAL_CODE;
    #endif

    // ####### SET OUTPUTS #######
    if ((speedL < lastSpeedL + 50 && speedL > lastSpeedL - 50) && (speedR < lastSpeedR + 50 && speedR > lastSpeedR - 50) && timeout < TIMEOUT) {
    #ifdef INVERT_R_DIRECTION
      throttlelr[1] = speedR;
    #else
      throttlelr[1] = -speedR;
    #endif
    #ifdef INVERT_L_DIRECTION
      throttlelr[0] = -speedL;
    #else
      throttlelr[0] = speedL;
    #endif
    }

    lastSpeedL = speedL;
    lastSpeedR = speedR;

    // ####### LOG TO CONSOLE #######
    consoleScope();

    if (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {
      set_bldc_motors(false);
      while (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {}
      buzzerFreq = 0;
      buzzerPattern = 0;
      for (int i = 0; i < 8; i++) {
        buzzerFreq = i;
        HAL_Delay(100);
      }
      HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, 0);
      while(1) {}
    }

    if (batteryVoltage < BAT_LOW_LVL1 && batteryVoltage > BAT_LOW_LVL2) {
      buzzerFreq = 5;
      buzzerPattern = 8;
    } else if  (batteryVoltage < BAT_LOW_LVL2 && batteryVoltage > BAT_LOW_DEAD) {
      buzzerFreq = 5;
      buzzerPattern = 1;
    } else if  (batteryVoltage < BAT_LOW_DEAD) {
      buzzerPattern = 0;
      set_bldc_motors(false);
      for (int i = 0; i < 8; i++) {
        buzzerFreq = i;
        HAL_Delay(100);
      }
      HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, 0);
      while(1) {}
    } else {
      buzzerFreq = 0;
      buzzerPattern = 0;
    }
  }
}

/** System Clock Configuration
*/
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL16;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection    = RCC_ADCPCLK2_DIV8;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

  /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}
