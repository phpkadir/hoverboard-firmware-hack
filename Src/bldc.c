#include <stdbool.h>
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "control.h"
#include "config.h"
#include "weaking.h"

uint8_t buzzerFreq = 0;
uint8_t buzzerPattern = 0;

const int pwm_res = 64000000 / 2 / PWM_FREQ; // = 2000

#define START_FREQ 0
#define END_FREQ 0

int calcWeakening(int pwm,int freq){ // never used
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

inline void blockPhaseCurrent(int pos, int u, int v, int *q) {
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
}

uint16_t offsetrl1   = 2048,
  offsetrl2   = 2048,
  offsetrr1   = 2048,
  offsetrr2   = 2048,
  offsetdcl   = 2048,
  offsetdcr   = 2048;

volatile unsigned long mainCounter = 0;

volatile float batteryVoltage = 40.0;

const int max_time = PWM_FREQ / 10; // never used

typedef void (*setMotorType)(int *hPhase);
void set_motor_r(int *hPhase){
  RIGHT_TIM->RIGHT_TIM_U = CLAMP(hPhase[0] + pwm_res / 2, 10, pwm_res-10);
  RIGHT_TIM->RIGHT_TIM_V = CLAMP(hPhase[1] + pwm_res / 2, 10, pwm_res-10);
  RIGHT_TIM->RIGHT_TIM_W = CLAMP(hPhase[2] + pwm_res / 2, 10, pwm_res-10);
}
void set_motor_l(int *hPhase){
  LEFT_TIM->LEFT_TIM_U = CLAMP(hPhase[0] + pwm_res / 2, 10, pwm_res-10);
  LEFT_TIM->LEFT_TIM_V = CLAMP(hPhase[1] + pwm_res / 2, 10, pwm_res-10);
  LEFT_TIM->LEFT_TIM_W = CLAMP(hPhase[2] + pwm_res / 2, 10, pwm_res-10);
}
setMotorType set_motor[2] = { //array for loop
  set_motor_l,
  set_motor_r
};
uint8_t get_pos_l(){
  return hall2pos[!(LEFT_HALL_W_PORT->IDR & LEFT_HALL_W_PIN)]
    [!(LEFT_HALL_V_PORT->IDR & LEFT_HALL_V_PIN)]
      [!(LEFT_HALL_U_PORT->IDR & LEFT_HALL_U_PIN)];
}
uint8_t get_pos_r(){
  return hall2pos[!(RIGHT_HALL_W_PORT->IDR & RIGHT_HALL_W_PIN)]
    [!(RIGHT_HALL_V_PORT->IDR & RIGHT_HALL_V_PIN)]
      [!(RIGHT_HALL_U_PORT->IDR & RIGHT_HALL_U_PIN)];
}
typedef uint8_t (*getPosType)();
getPosType get_pos[2]={
  get_pos_l,
  get_pos_r
};

void calibration_func();

void nullFunc(){}
void oldBuzzer(){
  if (buzzerFreq != 0 && (mainCounter / 5000) % (buzzerPattern + 1) == 0) {
    if (mainCounter % buzzerFreq == 0)
      HAL_GPIO_TogglePin(BUZZER_PORT, BUZZER_PIN);
  } else {
      HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, 0);
  }
}
volatile uint current_limit;
volatile int currentlr[2];
volatile int throttlelr[2];
volatile uint timer[2];
volatile uint8_t last_pos[2];
volatile uint phase_period[2];
volatile int blockcurlr[2];

volatile WeakingPtr currentWeaking = nullFuncWeak;

void brushless_countrol(){
  if((currentlr[0] = ABS(adc_buffer.dcl - offsetdcl) * MOTOR_AMP_CONV_DC_AMP) > current_limit || timeout > TIMEOUT)
    LEFT_TIM->BDTR &= ~TIM_BDTR_MOE;
  else
    LEFT_TIM->BDTR |= TIM_BDTR_MOE;

  if((currentlr[1] = ABS(adc_buffer.dcr - offsetdcr) * MOTOR_AMP_CONV_DC_AMP) > current_limit || timeout > TIMEOUT)
    RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE;
  else
    RIGHT_TIM->BDTR |= TIM_BDTR_MOE;
  //PWM part
  int phase[3];
  int wphase[3];
  //update PWM channels based on position
  for(int x = 0; x < 2; x++){
    uint8_t pos = get_pos[x];
    RetValWeak tmp = currentWeaking(throttlelr[x], phase_period[x],timer[x],currentlr[x]);
    blockPWM(tmp.pwm, pos, &phase[0], &phase[1], &phase[2]);
    blockPWM(tmp.weak, (pos+(tmp.pwm > 0?5:1)) % 6, &wphase[0], &wphase[1], &wphase[2]);
    for(int y = 0; y < 3; y++)
      phase[y] += wphase[y];
    set_motor[x](phase);
    //speed measurung
    timer[x]++;
    if(last_pos[x]!=pos){
      phase_period[x] = timer[x];
      timer[x] = 0;
    } else if(timer[x] > phase_period[x])
      timer[x] = phase_period[x];
  }
  blockPhaseCurrent(last_pos[0], adc_buffer.rl1 - offsetrl1, adc_buffer.rl2 - offsetrl2, &blockcurlr[0]); //Old shitty code
  blockPhaseCurrent(last_pos[1], adc_buffer.rr1 - offsetrr1, adc_buffer.rr2 - offsetrr2, &blockcurlr[1]); //Old shitty code
}

typedef void (*IsrPtr)();
volatile IsrPtr timer_brushless = calibration_func;
volatile IsrPtr buzzerFunc = nullFunc;

void calibration_func(){
  if(mainCounter < 1024) {  // calibrate ADC offsets
    offsetrl1 = (adc_buffer.rl1 + offsetrl1) / 2;
    offsetrl2 = (adc_buffer.rl2 + offsetrl2) / 2;
    offsetrr1 = (adc_buffer.rr1 + offsetrr1) / 2;
    offsetrr2 = (adc_buffer.rr2 + offsetrr2) / 2;
    offsetdcl = (adc_buffer.dcl + offsetdcl) / 2;
    offsetdcr = (adc_buffer.dcr + offsetdcr) / 2;
  }
  else{
    timer_brushless = brushless_countrol;
  }
}

void set_bldc_motors(bool enable){
  if(timer_brushless != calibration_func){
    if(enable)
      timer_brushless = brushless_countrol;
    else
      timer_brushless = nullFunc;
  }

}

//scan 8 channels with 2ADCs @ 20 clk cycles per sample
//meaning ~80 ADC clock cycles @ 8MHz until new DMA interrupt =~ 100KHz
//=640 cpu cycles
void DMA1_Channel1_IRQHandler() {
  DMA1->IFCR = DMA_IFCR_CTCIF1;
  mainCounter++;
  timer_brushless();
  buzzerFunc();
  //HAL_GPIO_WritePin(LED_PORT, LED_PIN, 1);
  //HAL_GPIO_WritePin(LED_PORT, LED_PIN, 0);
  if (!(mainCounter & 0x3FF))  // because you get float rounding errors if it would run every time every 1024th time
    batteryVoltage = batteryVoltage * 0.99 + ((float)adc_buffer.batt1 * ((float)BAT_CALIB_REAL_VOLTAGE / (float)BAT_CALIB_ADC)) * 0.01;
  // murks
}