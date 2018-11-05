#include "stm32f1xx_hal.h"
#include "weaking.h"
#include "bldc.h"
#include "config.h"

RetValWeak nullFuncWeak(int torque, uint period, uint cur_phase, int current){
  map()
  return (RetValWeak){ .pwm  = torque, .weak = 0};
}
RetValWeak turboBtn(int torque, uint period, uint cur_phase, int current){
  if(pwm>900 && period < 0)
    return (RetValWeak){ .pwm  = torque, .weak = 0};
  else
    return (RetValWeak){ .pwm  = torque, .weak = 0};
}
RetValWeak optWeaking(int torque, uint period, uint cur_phase, int current){
  return (RetValWeak){ .pwm  = PWM_MAX, .weak = 0};
}

const WeakStruct weakfunctions[] = {{nullFuncWeak,"!Weaked"},{turboBtn,"Turbo"}};
void set_weaking(int x){
  currentWeaking = weakfunctions[x].func;
}