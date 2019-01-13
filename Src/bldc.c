//FINAL V1.0
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "control.h"
#include "config.h"
#include "weaking.h"
#include "buzzertones.h"

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

inline void blockPWM(int pwm, int pos, int *u, int *v, int *w) {  // checked working
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

inline void blockPhaseCurrent(int pos, int u, int v, int *q) {  // unknown
  switch(pos) {
    case 0:
      *q = u - v;
      break;
    case 1:
      *q = u;
      break;
    case 2:
      *q = u;
      break;
    case 3:
      *q = v;
      break;
    case 4:
      *q = v;
      break;
    case 5:
      *q = -(u - v);
      break;
    default:
      *q = 0;
  }
}

uint16_t adc_offset[6] = {  // offests as array for looping adc
  ADC_MID,
  ADC_MID,
  ADC_MID,
  ADC_MID,
  ADC_MID,
  ADC_MID
};

volatile unsigned long mainCounter = 0;  // global time incremented by interrupt
volatile uint32_t battery_voltage = 1704<<10;// done use int
volatile int phase_period[2];  // the measured speed in 1/x
volatile unsigned int current_limit;  // dynamic Currentlimit
volatile int currentlr[2];  // Current for cutoff
volatile int throttlelr[2];  // throttle for calcing pwm and weakening
volatile unsigned int timer[2];  // timer for speed measuring
volatile uint8_t last_pos[2];  // for speed measuring and sensorless control
volatile int blockcurlr[2];  // Current for sensorles bldc
volatile int internal_phase_period[2];  // For internal calculations only  PRIVATE NOT IN C HEADER


unsigned long get_mainCounter(){
  return mainCounter;
}


typedef void (*setMotorType)(int *hPhase);
void set_motor_r(int *hPhase){
  RIGHT_TIM->RIGHT_TIM_U = CLAMP(hPhase[0] + PWM_RES / 2, 10, PWM_RES-10);
  RIGHT_TIM->RIGHT_TIM_V = CLAMP(hPhase[1] + PWM_RES / 2, 10, PWM_RES-10);
  RIGHT_TIM->RIGHT_TIM_W = CLAMP(hPhase[2] + PWM_RES / 2, 10, PWM_RES-10);
}
void set_motor_l(int *hPhase){
  LEFT_TIM->LEFT_TIM_U = CLAMP(hPhase[0] + PWM_RES / 2, 10, PWM_RES-10);
  LEFT_TIM->LEFT_TIM_V = CLAMP(hPhase[1] + PWM_RES / 2, 10, PWM_RES-10);
  LEFT_TIM->LEFT_TIM_W = CLAMP(hPhase[2] + PWM_RES / 2, 10, PWM_RES-10);
}
const setMotorType set_motor[2] = { //array for loop
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
const getPosType get_pos[2]={
  get_pos_l,
  get_pos_r
};

typedef void (*SetBoolFunc)(bool enable);
void set_tim_l(bool enable){
  if(enable)
    LEFT_TIM->BDTR |= TIM_BDTR_MOE;
  else
    LEFT_TIM->BDTR &= ~TIM_BDTR_MOE;
}
void set_tim_r(bool enable){
  if(enable)
    RIGHT_TIM->BDTR |= TIM_BDTR_MOE;
  else
    RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE;
}

const SetBoolFunc set_tim_lr[2] = {  // disable/enable motors
  set_tim_l,
  set_tim_r
};

void nullFunc(){}  // Function for empty funktionpointer becasue Jump NULL != ret

typedef void (*SetPhaseType)(int phase);
void set_phase_l(int phase){
#if defined(INVERT_L_DIRECTION)  // for other hardware needs this function to be rewritten
  phase_period[0] = -phase;
#else
  phase_period[0] = phase;
#endif
}
void set_phase_r(int phase){
#if defined(INVERT_R_DIRECTION)
  phase_period[1] = -phase;
#else
  phase_period[1] = phase;
#endif
}

const SetPhaseType set_phase_lr[2] = {  // disable/enable motors
  set_phase_l,
  set_phase_r
};

volatile WeakingPtr currentWeaking = nullFuncWeak;  // Pointer for calculing fealdweakening and pwm

#ifdef TIMING_ENABLE
volatile TimingPtr currentTiming = no_timing;  // Pointer for calculating the timing of the motor (to prebuild the electric fields)
#endif

uint8_t next_pos(uint8_t oldpos, int8_t direction){
  switch(direction){
    case -1:
      return (oldpos + 5) % 6;  // -1
    case 1:
      return (oldpos + 1) % 6;  // +1
    default:
      return oldpos;
  }
}

typedef void (*IsrPtr)();
volatile IsrPtr timer_brushless = nullFunc;
volatile IsrPtr buzzerFunc = nullFunc;

void sensored_brushless_countrol(){
  //PWM part
  int phase[3];
  int wphase[3];
  //update PWM channels based on position
  for(int x = 0; x < 2; x++){
    uint8_t real_pos = get_pos[x](), timing_pos;
    timer[x]++;
    if(last_pos[x] != real_pos){
      if(next_pos(last_pos[x],1) == real_pos)
        set_phase_lr[x](internal_phase_period[x] = timer[x]);
      else
        set_phase_lr[x](internal_phase_period[x] = -timer[x]);
      timer[x] = 0;
      last_pos[x] = real_pos;
    } else if(timer[x] > abs(internal_phase_period[x]))
      set_phase_lr[x](internal_phase_period[x] = SIGN(internal_phase_period[x])*timer[x]);
    set_tim_lr[x]((currentlr[x] = ABS(adc_array[5-x] - adc_offset[5-x]) * MOTOR_AMP_CONV_DC_AMP) <= current_limit);
    RetValWeak tmp = currentWeaking(throttlelr[x],
      internal_phase_period[x],
      timer[x],
      currentlr[x]);
    if(tmp.timing)
      timing_pos = next_pos(real_pos,
        SIGN(internal_phase_period[x]));
    else
      timing_pos = real_pos;
    blockPWM(tmp.pwm,
      timing_pos,
      &phase[0],
      &phase[1],
      &phase[2]);
    blockPWM(tmp.weak,
      (timing_pos + (internal_phase_period[x] > 0 ? 5 : 1)) % 6,
      &wphase[0],
      &wphase[1],
      &wphase[2]);
    for(int y = 0; y < 3; y++)
      phase[y] += wphase[y];
    set_motor[x](phase);
    blockPhaseCurrent(last_pos[x], adc_array[3-2*x] - adc_offset[3-2*x], adc_array[4-2*x] - adc_offset[4-2*x], &blockcurlr[x]); //Old shitty code
  }
}

void calibration_func();  // for correct var accessing and do not set to public

void stop_buzzer(){
  buzzerFunc = nullFunc;
  HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, 0);
}

void bldc_start_calibration(){
  mainCounter = 0;
  timer_brushless = calibration_func;
}

void bldc_load_calibration(void* buffer){

}

void set_buzzer(void* buzzerfunc){
  if(buzzerFunc == buzzerfunc)
    return;
  set_buzzerStart(mainCounter);
  buzzerFunc = buzzerfunc;
}

void calibration_func(){
  if(mainCounter < 1024)  // calibrate ADC offsets
    for(int x = 0; x < 6; x++)
      adc_offset[x] = (adc_array[x] + adc_offset[x]) / 2;
  else
    timer_brushless = sensored_brushless_countrol;
}

void blink_led(){
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, mainCounter/PWM_FREQ%2);
}

void set_bldc_motors(bool enable){
  if(timer_brushless != calibration_func){  // if calibration is running do NOT enable Brushless motors
    if(enable) {
      timer_brushless = sensored_brushless_countrol;
    }
    else {
      timer_brushless = nullFunc;
      int phase[3];  // disable output
      for(int x = 0; x < 2; x++){
        blockPWM(0, last_pos[x], &phase[0], &phase[1], &phase[2]);
        set_motor[x](phase);
      }
    }
  }
}

void set_bldc_to_led(){
  timer_brushless = blink_led;
}

void set_throttle(int left,int right){  // get set access for the throttle and switchin the direction of one motor
#if defined(INVERT_L_DIRECTION)  // for other hardware needs this function to be rewritten
  throttlelr[0] = CLAMP(-left,-THROTTLE_MAX,THROTTLE_MAX);
#else
  throttlelr[0] = CLAMP(left,-THROTTLE_MAX,THROTTLE_MAX);
#endif
#if defined(INVERT_R_DIRECTION)
  throttlelr[1] = CLAMP(-right,-THROTTLE_MAX,THROTTLE_MAX);
#else
  throttlelr[1] = CLAMP(right,-THROTTLE_MAX,THROTTLE_MAX);
#endif
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
  if (!(mainCounter & 0x3F))  // because you get float rounding errors if it would run every time every 1024th time
    battery_voltage = battery_voltage * ((1<<10)-1) / (1<<10) + adc_buffer.batt1;
  // murks
}