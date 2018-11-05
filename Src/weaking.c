#include "stm32f1xx_hal.h"
#include "weaking.h"
#include "bldc.h"
int nullFuncWeak(int pwm, uint period, uint cur_phase, int current){
  return 0;
}
int turboBtn(int pwm, uint period, uint cur_phase, int current){
  if(pwm>900)
    return WEAKING_PWM_MAX;
  else
    return 0;
}

const WeakStruct weakfunctions[] = {{nullFuncWeak,"!Weaked"},{turboBtn,"Turbo"}};
void set_weaking(int x){
  currentWeaking = weakfunctions[x].func;
}