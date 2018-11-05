#include "stm32f1xx_hal.h"
#include "weaking.h"
#include "bldc.h"

RetValWeak nullFuncWeak(int pwm, uint period, uint cur_phase, int current){
  return {0,0};
}
RetValWeak turboBtn(int pwm, uint period, uint cur_phase, int current){
  if(pwm>900 && period < 0)
    return {pwm,WEAKING_PWM_MAX};
  else
    return {pwm,0};
}
RetValWeak optWeaking(int pwm, uint period, uint cur_phase, int current){
  
}

const WeakStruct weakfunctions[] = {{nullFuncWeak,"!Weaked"},{turboBtn,"Turbo"}};
void set_weaking(int x){
  currentWeaking = weakfunctions[x].func;
}