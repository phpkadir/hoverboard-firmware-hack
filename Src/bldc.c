
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"


volatile int posl = 0;
volatile int posr = 0;
volatile unsigned int freql;  // for later inplementation
volatile unsigned int freqr;  // for later inplementation
volatile int pwml = 0;
volatile int pwmr = 0;
volatile int weakl = 0;
volatile int weakr = 0;

int lst_posl = 0;
int lst_isr_posl = 0;
int lst_posr = 0;
int lst_isr_posr = 0;
int isr_counter = 0;

extern volatile int speed;

extern volatile adc_buf_t adc_buffer;

extern volatile uint32_t timeout;

uint8_t buzzerFreq = 0;
uint8_t buzzerPattern = 0;

uint8_t enable = 0;

const int pwm_res = 64000000 / 2 / PWM_FREQ; // = 2000

int calcWeakening(int pwm,int freq){
  if (freq < START_FREQ) return 0;
  if (freq >= END_FREQ)
    return pwm*FEALD_WEAKENING_MAX/100;
  else
    return (freq-START_FREQ)*pwm*FEALD_WEAKENING_MAX/100/(END_FREQ-START_FREQ);
}

const uint8_t hall2pos[2][2][2] = {
  {
    {
      2,
      2
    },
    {
      4,
      3
    }
  },
  {
    {
      0,
      1
    },
    {
      5,
      2
    }
  }
};

inline void blockPWM(int pwm, int pos, int *u, int *v, int *w) {
  switch(pos) {
    case 0:
      *u = 0;
      *v = pwm;
      *w = -pwm;
      break;
    case 1:
      *u = -pwm;
      *v = pwm;
      *w = 0;
      break;
    case 2:
      *u = -pwm;
      *v = 0;
      *w = pwm;
      break;
    case 3:
      *u = 0;
      *v = -pwm;
      *w = pwm;
      break;
    case 4:
      *u = pwm;
      *v = -pwm;
      *w = 0;
      break;
    case 5:
      *u = pwm;
      *v = 0;
      *w = -pwm;
      break;
    default:
      *u = 0;
      *v = 0;
      *w = 0;
  }
}

//int curl = 0;
/*inline void blockPhaseCurrent(int pos, int u, int v, int *q) {
  switch(pos) {
    case 0:
      *q = u - v;
      // *u = 0;
      // *v = pwm;
      // *w = -pwm;
      break;
    case 1:
      *q = u;
      // *u = -pwm;
      // *v = pwm;
      // *w = 0;
      break;
    case 2:
      *q = u;
      // *u = -pwm;
      // *v = 0;
      // *w = pwm;
      break;
    case 3:
      *q = v;
      // *u = 0;
      // *v = -pwm;
      // *w = pwm;
      break;
    case 4:
      *q = v;
      // *u = pwm;
      // *v = -pwm;
      // *w = 0;
      break;
    case 5:
      *q = -(u - v);
      // *u = pwm;
      // *v = 0;
      // *w = -pwm;
      break;
    default:
      *q = 0;
      // *u = 0;
      // *v = 0;
      // *w = 0;
  }
}*/

uint16_t buzzerTimer        = 0;

int offsetcount = 0;
int offsetrl1   = 2000;
int offsetrl2   = 2000;
int offsetrr1   = 2000;
int offsetrr2   = 2000;
int offsetdcl   = 2000;
int offsetdcr   = 2000;

float batteryVoltage = 40.0;


// int errorl = 0;
// int kp = 5;
// volatile int cmdl = 0;

const int max_time = PWM_FREQ / 10;
volatile int vel = 0;

//scan 8 channels with 2ADCs @ 20 clk cycles per sample
//meaning ~80 ADC clock cycles @ 8MHz until new DMA interrupt =~ 100KHz
//=640 cpu cycles
void DMA1_Channel1_IRQHandler() {
  isr_counter++; // I hope its a timer interrupt :)
  DMA1->IFCR = DMA_IFCR_CTCIF1;
  // HAL_GPIO_WritePin(LED_PORT, LED_PIN, 1);

  if(offsetcount < 1000) {  // calibrate ADC offsets
    offsetcount++;
    offsetrl1 = (adc_buffer.rl1 + offsetrl1) / 2;
    offsetrl2 = (adc_buffer.rl2 + offsetrl2) / 2;
    offsetrr1 = (adc_buffer.rr1 + offsetrr1) / 2;
    offsetrr2 = (adc_buffer.rr2 + offsetrr2) / 2;
    offsetdcl = (adc_buffer.dcl + offsetdcl) / 2;
    offsetdcr = (adc_buffer.dcr + offsetdcr) / 2;
    return;
  }

  if (buzzerTimer % 100 == 0) {
    batteryVoltage = batteryVoltage * 0.999 + ((float)adc_buffer.batt1 * ADC_BATTERY_VOLT) * 0.001;
  }


  #ifdef BEEPS_BACKWARD
    if (speed < -50 && enable == 1) {
      buzzerFreq = 5;
      buzzerPattern = 1;
    } else if (enable == 1) {
      buzzerFreq = 0;
      buzzerPattern = 1;
    }
  #endif


  //disable PWM when current limit is reached (current chopping)
  if(ABS((adc_buffer.dcl - offsetdcl) * MOTOR_AMP_CONV_DC_AMP) > DC_CUR_LIMIT || timeout > TIMEOUT || enable == 0)
    LEFT_TIM->BDTR &= ~TIM_BDTR_MOE;
  else
    LEFT_TIM->BDTR |= TIM_BDTR_MOE;

  if(ABS((adc_buffer.dcr - offsetdcr) * MOTOR_AMP_CONV_DC_AMP)  > DC_CUR_LIMIT || timeout > TIMEOUT || enable == 0)
    RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE;
  else
    RIGHT_TIM->BDTR |= TIM_BDTR_MOE;

  //determine next position based on hall sensors
  posl = hall2pos[!(LEFT_HALL_W_PORT->IDR & LEFT_HALL_W_PIN)][!(LEFT_HALL_V_PORT->IDR & LEFT_HALL_V_PIN)][!(LEFT_HALL_U_PORT->IDR & LEFT_HALL_U_PIN)];
  posr = hall2pos[!(RIGHT_HALL_W_PORT->IDR & RIGHT_HALL_W_PIN)][!(RIGHT_HALL_V_PORT->IDR & RIGHT_HALL_V_PIN)][!(RIGHT_HALL_U_PORT->IDR & RIGHT_HALL_U_PIN)];

  if (posl != lst_posl) {
    freql = isr_counter - lst_isr_posl;
    lst_isr_posl = isr_counter;
  }
  if (posr != lst_posr) {
    freqr = isr_counter - lst_isr_posr;
    lst_isr_posr = isr_counter;
  }
  //blockPhaseCurrent(posl, adc_buffer.rl1 - offsetrl1, adc_buffer.rl2 - offsetrl2, &curl);

  //setScopeChannel(2, (adc_buffer.rl1 - offsetrl1) / 8);
  //setScopeChannel(3, (adc_buffer.rl2 - offsetrl2) / 8);

  //create square wave for buzzer
  buzzerTimer++;
  if (buzzerFreq != 0 && (buzzerTimer / 5000) % (buzzerPattern + 1) == 0)
    if (buzzerTimer % buzzerFreq == 0)
      HAL_GPIO_TogglePin(BUZZER_PORT, BUZZER_PIN);
  else
      HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, 0);

  //update PWM channels based on position
  int ul, vl, wl, ur, vr, wr;
  blockPWM(pwml, posl, &ul, &vl, &wl);
  blockPWM(pwmr, posr, &ur, &vr, &wr);

  int weakul, weakvl, weakwl, weakur, weakvr, weakwr;
  blockPWM(calcWeakening(pwml, freql), (pwml > 0) ? (posl+5) % 6 : (posl+1) % 6, &weakul, &weakvl, &weakwl);
  blockPWM(calcWeakening(pwmr, freqr), (pwmr > 0) ? (posr+5) % 6 : (posr+1) % 6, &weakur, &weakvr, &weakwr);

  LEFT_TIM->LEFT_TIM_U = CLAMP(ul + weakul + pwm_res / 2, 10, pwm_res-10);
  LEFT_TIM->LEFT_TIM_V = CLAMP(vl + weakvl + pwm_res / 2, 10, pwm_res-10);
  LEFT_TIM->LEFT_TIM_W = CLAMP(wl + weakwl + pwm_res / 2, 10, pwm_res-10);

  RIGHT_TIM->RIGHT_TIM_U = CLAMP(ur + weakur + pwm_res / 2, 10, pwm_res-10);
  RIGHT_TIM->RIGHT_TIM_V = CLAMP(vr + weakvr + pwm_res / 2, 10, pwm_res-10);
  RIGHT_TIM->RIGHT_TIM_W = CLAMP(wr + weakwr + pwm_res / 2, 10, pwm_res-10);
}
