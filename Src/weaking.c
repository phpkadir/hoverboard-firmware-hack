#include "stm32f1xx_hal.h"
#include "weaking.h"
#include "bldc.h"
#include "config.h"

RetValWeak longRange(int torque, uint period, uint cur_phase, int current){
  return (RetValWeak){ .pwm  = torque, .weak = 0};
}
RetValWeak STVO6kmh(int torque, uint period, uint cur_phase, int current){
  return (RetValWeak){ .pwm  = torque/8, .weak = 0};
}

RetValWeak nullFuncWeak(int torque, uint period, uint cur_phase, int current){
  return (RetValWeak){ .pwm  = 0, .weak = 0};
}
RetValWeak fastMode(int torque, uint period, uint cur_phase, int current){//todo same algorythmus than optweaking but less agressive and more efficient
  if(pwm>950 && period < 0)
    return (RetValWeak){ .pwm  = PWM_MAX, .weak = WEAKING_PWM_MAX};
  else
    return (RetValWeak){ .pwm  = torque, .weak = 0};
}
RetValWeak optWeaking(int torque, uint period, uint cur_phase, int current){//todo
  return (RetValWeak){ .pwm  = PWM_MAX, .weak = 0};
}

const WeakStruct weakfunctions[] = {{nullFuncWeak,"Off", 0},{STVO6kmh,"STVO", 5},{longRange,"LongRange", 15},{fastMode,"fast",20},{optWeaking,"noLimit",60}};
void set_weaking(int x){
  currentWeaking = weakfunctions[x].func;
  current_limit = weakfunctions[x].cur_limit;
}
char* get_weaking_name(int x){
  return weakfunctions[x].name;
}