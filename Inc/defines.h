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

#pragma once
#include "stm32f1xx_hal.h"

#define LEFT_HALL_U_PIN GPIO_PIN_5
#define LEFT_HALL_V_PIN GPIO_PIN_6
#define LEFT_HALL_W_PIN GPIO_PIN_7

#define LEFT_HALL_U_PORT GPIOB
#define LEFT_HALL_V_PORT GPIOB
#define LEFT_HALL_W_PORT GPIOB

#define RIGHT_HALL_U_PIN GPIO_PIN_10
#define RIGHT_HALL_V_PIN GPIO_PIN_11
#define RIGHT_HALL_W_PIN GPIO_PIN_12

#define RIGHT_HALL_U_PORT GPIOC
#define RIGHT_HALL_V_PORT GPIOC
#define RIGHT_HALL_W_PORT GPIOC

#define LEFT_TIM TIM8
#define LEFT_TIM_U CCR1
#define LEFT_TIM_UH_PIN GPIO_PIN_6
#define LEFT_TIM_UH_PORT GPIOC
#define LEFT_TIM_UL_PIN GPIO_PIN_7
#define LEFT_TIM_UL_PORT GPIOA
#define LEFT_TIM_V CCR2
#define LEFT_TIM_VH_PIN GPIO_PIN_7
#define LEFT_TIM_VH_PORT GPIOC
#define LEFT_TIM_VL_PIN GPIO_PIN_0
#define LEFT_TIM_VL_PORT GPIOB
#define LEFT_TIM_W CCR3
#define LEFT_TIM_WH_PIN GPIO_PIN_8
#define LEFT_TIM_WH_PORT GPIOC
#define LEFT_TIM_WL_PIN GPIO_PIN_1
#define LEFT_TIM_WL_PORT GPIOB

#define RIGHT_TIM TIM1
#define RIGHT_TIM_U CCR1
#define RIGHT_TIM_UH_PIN GPIO_PIN_8
#define RIGHT_TIM_UH_PORT GPIOA
#define RIGHT_TIM_UL_PIN GPIO_PIN_13
#define RIGHT_TIM_UL_PORT GPIOB
#define RIGHT_TIM_V CCR2
#define RIGHT_TIM_VH_PIN GPIO_PIN_9
#define RIGHT_TIM_VH_PORT GPIOA
#define RIGHT_TIM_VL_PIN GPIO_PIN_14
#define RIGHT_TIM_VL_PORT GPIOB
#define RIGHT_TIM_W CCR3
#define RIGHT_TIM_WH_PIN GPIO_PIN_10
#define RIGHT_TIM_WH_PORT GPIOA
#define RIGHT_TIM_WL_PIN GPIO_PIN_15
#define RIGHT_TIM_WL_PORT GPIOB

// #define LEFT_DC_CUR_ADC ADC1
// #define LEFT_U_CUR_ADC ADC1
// #define LEFT_V_CUR_ADC ADC1

#define LEFT_DC_CUR_PIN GPIO_PIN_0
#define LEFT_U_CUR_PIN GPIO_PIN_0
#define LEFT_V_CUR_PIN GPIO_PIN_3

#define LEFT_DC_CUR_PORT GPIOC
#define LEFT_U_CUR_PORT GPIOA
#define LEFT_V_CUR_PORT GPIOC

// #define RIGHT_DC_CUR_ADC ADC2
// #define RIGHT_U_CUR_ADC ADC2
// #define RIGHT_V_CUR_ADC ADC2

#define RIGHT_DC_CUR_PIN GPIO_PIN_1
#define RIGHT_U_CUR_PIN GPIO_PIN_4
#define RIGHT_V_CUR_PIN GPIO_PIN_5

#define RIGHT_DC_CUR_PORT GPIOC
#define RIGHT_U_CUR_PORT GPIOC
#define RIGHT_V_CUR_PORT GPIOC

// #define DCLINK_ADC ADC3
// #define DCLINK_CHANNEL
#define DCLINK_PIN GPIO_PIN_2
#define DCLINK_PORT GPIOC
// #define DCLINK_PULLUP 30000
// #define DCLINK_PULLDOWN 1000

#define LED_PIN GPIO_PIN_2
#define LED_PORT GPIOB

#define BUZZER_PIN GPIO_PIN_4
#define BUZZER_PORT GPIOA

#define SWITCH_PIN GPIO_PIN_1
#define SWITCH_PORT GPIOA

#define OFF_PIN GPIO_PIN_5
#define OFF_PORT GPIOA

#define BUTTON_PIN GPIO_PIN_1
#define BUTTON_PORT GPIOA

#define CHARGER_PIN GPIO_PIN_12
#define CHARGER_PORT GPIOA

#define DELAY_TIM_FREQUENCY_US 1000000

#define MOTOR_AMP_CONV_DC_AMP 50
#define ADC_BATTERY_VOLT      0.02647435897435897435897435897436

#define MILLI_R (R * 1000)
#define MILLI_PSI (PSI * 1000)
#define MILLI_V (V * 1000)

#define NO 0
#define YES 1
#define ABS(a) (((a) < 0.0) ? -(a) : (a))
#define LIMIT(x, lowhigh) (((x) > (lowhigh)) ? (lowhigh) : (((x) < (-lowhigh)) ? (-lowhigh) : (x)))
#define SAT(x, lowhigh) (((x) > (lowhigh)) ? (1.0) : (((x) < (-lowhigh)) ? (-1.0) : (0.0)))
#define SAT2(x, low, high) (((x) > (high)) ? (1.0) : (((x) < (low)) ? (-1.0) : (0.0)))
#define STEP(from, to, step) (((from) < (to)) ? (MIN((from) + (step), (to))) : (MAX((from) - (step), (to))))
#define DEG(a) ((a)*M_PI / 180.0)
#define RAD(a) ((a)*180.0 / M_PI)
#define SIGN(a) (((a) < 0.0) ? (-1.0) : (((a) > 0.0) ? (1.0) : (0.0)))
#define CLAMP(x, low, high) (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))
#define SCALE(value, high, max) MIN(MAX(((max) - (value)) / ((max) - (high)), 0.0), 1.0)
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN3(a, b, c) MIN(a, MIN(b, c))
#define MAX3(a, b, c) MAX(a, MAX(b, c))

#define BAT_LOW_LVL1     (BAT_CELL_CNT*BAT_LOW1_CELL)       // gently beeps at this voltage level. ~3.6V/cell
#define BAT_LOW_LVL2     (BAT_CELL_CNT*BAT_LOW2_CELL)       // your battery is almost empty. Charge now! ~3.3V/cell
#define BAT_LOW_DEAD     (BAT_CELL_CNT*BAT_LOW_DEAD_CELL)       // undervoltage lockout. ~3.1V/cell
#define BAT_FULL         (BAT_CELL_CNT*BAT_FULL_CELL)
#define BAT_RATED        (BAT_CELL_CNT*BAT_RATED_CELL)
#define BATTERY_VOLTAGE2ADC12(x) (uint32_t)((float)((float)x / ((float)BAT_CALIB_REAL_VOLTAGE / (float)BAT_CALIB_ADC))*1024.0f)
#define ADC122BATTERY_VOLTAGE(x) ((float)(x)*((float)BAT_CALIB_REAL_VOLTAGE / (float)BAT_CALIB_ADC)/1024.0f)



#define PWM_RES (64000000 / 2 / PWM_FREQ) /* 2000 */
#define LIMIT_CURRENT(x) if(current_limit > x) current_limit = x;
#define IN_RANGE(x, low, up) (((x) >= (low)) && ((x) <= (up)))

typedef struct {
  uint16_t rr1;
  uint16_t rr2;
  uint16_t rl1;
  uint16_t rl2;
  uint16_t dcr;
  uint16_t dcl;
  uint16_t batt1;
  uint16_t l_tx2; //7
  uint16_t bat1;
  uint16_t l_rx2; //9
} adc_buf_t;
