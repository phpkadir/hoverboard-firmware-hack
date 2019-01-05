//ALPHA V0.1
#include <stdbool.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "control.h"
#include "config.h"
#include "weaking.h"
#include "buzzertones.h"
#include "timing.h"

#define pwm_res (64000000 / 2 / PWM_FREQ) /* 2000 */

const uint8_t hall2pos[2][2][2] = {  // unchecked
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

uint16_t adc_offset[6] = {  // offests as array for looping adc
  ADC_MID,
  ADC_MID,
  ADC_MID,
  ADC_MID,
  ADC_MID,
  ADC_MID
};
volatile unsigned long mainCounter = 0;  // global time incremented by interrupt

unsigned long get_mainCounter(){
  return mainCounter;
}

volatile float batteryVoltage = 40.0;  // measured batvoltage as float TODO: use int

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

volatile int phase_period[2];  // the measured speed in 1/x

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

volatile unsigned int current_limit;  // dynamic Currentlimit
volatile int currentlr[2];  // Current for cutoff
volatile int throttlelr[2];  // throttle for calcing pwm and weakening
volatile unsigned int timer[2];  // timer for speed measuring
volatile uint8_t last_pos[2];  // for speed measuring and sensorless control
volatile int blockcurlr[2];  // Current for sensorles bldc
volatile int internal_phase_period[2];  // For internal calculations only  PRIVATE NOT IN C HEADER
volatile WeakingPtr currentWeaking = nullFuncWeak;  // Pointer for calculing fealdweakening and pwm

#ifdef TIMING_ENABLE
volatile TimingPtr currentTiming = no_timing;  // Pointer for calculating the timing of the motor (to prebuild the electric fields)
#endif

uint8_t next_pos(uint8_t oldpos, int8_t direction){
  switch(direction){
    case -1:
      return (oldpos - 1) == -1 ? 5 : (oldpos - 1);
    case 1:
      return (oldpos + 1) == 6 ? 0 : (oldpos + 1);
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
    set_tim_lr[x]((currentlr[x] = ABS(adc_array[5-x] - adc_offset[5-x]) * MOTOR_AMP_CONV_DC_AMP) <= current_limit);
    //set_tim_lr[x](true); // ok for testing
    uint8_t real_pos = get_pos[x];
    #ifdef TIMING_ENABLE
    uint8_t timing_pos;
    if(currentTiming(internal_phase_period[x],
        timer[x],
        throttlelr[x]))
      timing_pos = next_pos(real_pos,
        SIGN(internal_phase_period[x]));
    else
      timing_pos = real_pos;  // ifdef true end else else its always calced
    #else
    #define timing_pos real_pos
    #endif
    RetValWeak tmp = currentWeaking(throttlelr[x],
      internal_phase_period[x],
      timer[x],
      currentlr[x]);
    blockPWM(tmp.pwm,
      timing_pos,
      &phase[0],
      &phase[1],
      &phase[2]);
    blockPWM(tmp.weak,
      (timing_pos + (tmp.pwm > 0 ? 5 : 1)) % 6,
      &wphase[0],
      &wphase[1],
      &wphase[2]);
    for(int y = 0; y < 3; y++)
      phase[y] += wphase[y];
    set_motor[x](phase);
    //speed measurung
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
    blockPhaseCurrent(last_pos[x], adc_array[3-2*x] - adc_offset[3-2*x], adc_array[4-2*x] - adc_offset[4-2*x], &blockcurlr[x]); //Old shitty code
  }
}


// fauth
int offsetcount = 0;
int offsetrl1   = 2000;
int offsetrl2   = 2000;
int offsetrr1   = 2000;
int offsetrr2   = 2000;
int offsetdcl   = 2000;
int offsetdcr   = 2000;

const uint8_t hall_to_pos[8] = {
    0,
    0,
    2,
    1,
    4,
    5,
    3,
    0,
};

void fauth_brushless_control(){
  if(offsetcount < 1000) {  // calibrate ADC offsets
    offsetrl1 = (adc_buffer.rl1 + offsetrl1) / 2;
    offsetrl2 = (adc_buffer.rl2 + offsetrl2) / 2;
    offsetrr1 = (adc_buffer.rr1 + offsetrr1) / 2;
    offsetrr2 = (adc_buffer.rr2 + offsetrr2) / 2;
    offsetdcl = (adc_buffer.dcl + offsetdcl) / 2;
    offsetdcr = (adc_buffer.dcr + offsetdcr) / 2;
    return;
  }

  //disable PWM when current limit is reached (current chopping)
  if(ABS((adc_buffer.dcl - offsetdcl) * MOTOR_AMP_CONV_DC_AMP) > DC_CUR_LIMIT) {
    LEFT_TIM->BDTR &= ~TIM_BDTR_MOE;
    //HAL_GPIO_WritePin(LED_PORT, LED_PIN, 1);
  } else {
    LEFT_TIM->BDTR |= TIM_BDTR_MOE;
    //HAL_GPIO_WritePin(LED_PORT, LED_PIN, 0);
  }

  if(ABS((adc_buffer.dcr - offsetdcr) * MOTOR_AMP_CONV_DC_AMP)  > DC_CUR_LIMIT) {
    RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE;
  } else {
    RIGHT_TIM->BDTR |= TIM_BDTR_MOE;
  }

  int ul, vl, wl;
  int ur, vr, wr;

  //determine next position based on hall sensors
  uint8_t hall_ul = !(LEFT_HALL_U_PORT->IDR & LEFT_HALL_U_PIN);
  uint8_t hall_vl = !(LEFT_HALL_V_PORT->IDR & LEFT_HALL_V_PIN);
  uint8_t hall_wl = !(LEFT_HALL_W_PORT->IDR & LEFT_HALL_W_PIN);

  uint8_t hall_ur = !(RIGHT_HALL_U_PORT->IDR & RIGHT_HALL_U_PIN);
  uint8_t hall_vr = !(RIGHT_HALL_V_PORT->IDR & RIGHT_HALL_V_PIN);
  uint8_t hall_wr = !(RIGHT_HALL_W_PORT->IDR & RIGHT_HALL_W_PIN);

  uint8_t halll = hall_ul * 1 + hall_vl * 2 + hall_wl * 4;
  uint8_t posl          = hall_to_pos[halll];
  posl += 2;
  posl %= 6;

  uint8_t hallr = hall_ur * 1 + hall_vr * 2 + hall_wr * 4;
  uint8_t posr          = hall_to_pos[hallr];
  posr += 2;
  posr %= 6;

  blockPhaseCurrent(posl, adc_buffer.rl1 - offsetrl1, adc_buffer.rl2 - offsetrl2, &currentlr[0]);

  //setScopeChannel(2, (adc_buffer.rl1 - offsetrl1) / 8);
  //setScopeChannel(3, (adc_buffer.rl2 - offsetrl2) / 8);


  // uint8_t buzz(uint16_t *notes, uint32_t len){
    // static uint32_t counter = 0;
    // static uint32_t timer = 0;
    // if(len == 0){
        // return(0);
    // }
    
    // struct {
        // uint16_t freq : 4;
        // uint16_t volume : 4;
        // uint16_t time : 8;
    // } note = notes[counter];
    
    // if(timer / 500 == note.time){
        // timer = 0;
        // counter++;
    // }
    
    // if(counter == len){
        // counter = 0;
    // }

    // timer++;
    // return(note.freq);
  // }


  //update PWM channels based on position
  blockPWM(throttlelr[0], posl, &ul, &vl, &wl);
  blockPWM(throttlelr[1], posr, &ur, &vr, &wr);

  int weakul, weakvl, weakwl;
  if (throttlelr[0] > 0) {
    blockPWM(0, (posl+5) % 6, &weakul, &weakvl, &weakwl);
  } else {
    blockPWM(-0, (posl+1) % 6, &weakul, &weakvl, &weakwl);
  }
  ul += weakul;
  vl += weakvl;
  wl += weakwl;

  int weakur, weakvr, weakwr;
  if (throttlelr[1] > 0) {
    blockPWM(0, (posr+5) % 6, &weakur, &weakvr, &weakwr);
  } else {
    blockPWM(-0, (posr+1) % 6, &weakur, &weakvr, &weakwr);
  }
  ur += weakur;
  vr += weakvr;
  wr += weakwr;

  LEFT_TIM->LEFT_TIM_U = CLAMP(ul + pwm_res / 2, 10, pwm_res-10);
  LEFT_TIM->LEFT_TIM_V = CLAMP(vl + pwm_res / 2, 10, pwm_res-10);
  LEFT_TIM->LEFT_TIM_W = CLAMP(wl + pwm_res / 2, 10, pwm_res-10);

  RIGHT_TIM->RIGHT_TIM_U = CLAMP(ur + pwm_res / 2, 10, pwm_res-10);
  RIGHT_TIM->RIGHT_TIM_V = CLAMP(vr + pwm_res / 2, 10, pwm_res-10);
  RIGHT_TIM->RIGHT_TIM_W = CLAMP(wr + pwm_res / 2, 10, pwm_res-10);
}

void enable_fauth(){
  timer_brushless = fauth_brushless_control;
}

// end fauth


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

void set_throttle(int left,int right){  // get set access for the throttle and switchin the direction of one motor
#if defined(INVERT_L_DIRECTION)  // for other hardware needs this function to be rewritten
  throttlelr[0] = -left;
#else
  throttlelr[0] = CLAMP(left,-1000,1000);
#endif
#if defined(INVERT_R_DIRECTION)
  throttlelr[1] = -right,-1000,1000);
#else
  throttlelr[1] = right;
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
  if (!(mainCounter & 0x3FF))  // because you get float rounding errors if it would run every time every 1024th time
    batteryVoltage = batteryVoltage * 0.99 + ((float)adc_buffer.batt1 * ((float)BAT_CALIB_REAL_VOLTAGE / (float)BAT_CALIB_ADC)) * 0.01;
  // murks
}