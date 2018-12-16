#include <stdbool.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "weaking.h"
#include "bldc.h"
#include "config.h"

RetValWeak longRange(int torque, unsigned int period, unsigned int cur_phase, int current){
  return (RetValWeak){ .pwm  = torque, .weak = 0};
}
RetValWeak STVO6kmh(int torque, unsigned int period, unsigned int cur_phase, int current){
  return (RetValWeak){ .pwm  = torque/8, .weak = 0};
}

RetValWeak nullFuncWeak(int torque, unsigned int period, unsigned int cur_phase, int current){
  return (RetValWeak){ .pwm  = 0, .weak = 0};
}
RetValWeak fastMode(int torque, unsigned int period, unsigned int cur_phase, int current){//todo same algorythmus than optweaking but less agressive and more efficient
  if(torque>950 && period < 0)
    return (RetValWeak){ .pwm  = PWM_MAX, .weak = WEAKING_PWM_MAX};
  else
    return (RetValWeak){ .pwm  = torque, .weak = 0};
}
RetValWeak optWeaking(int torque, unsigned int period, unsigned int cur_phase, int current){//todo
  return (RetValWeak){ .pwm  = PWM_MAX, .weak = 0};
}

const WeakStruct weakfunctions[] = {
  {nullFuncWeak," Off", 0},
  {STVO6kmh,"STVO", 5},
  {longRange,"LoRa", 15},
  {fastMode,"Fast",20},
  {optWeaking,"Race",60}
};
void set_weaking(int x){
  currentWeaking = weakfunctions[x].func;
  current_limit = weakfunctions[x].cur_limit;
}
bool calc_timing(int lst_phase, int cur_phase, int throttle){ // todo timing algorythms
  return false;
}
bool no_timing(int lst_phase, int cur_phase, int throttle){
  return false;
}

const char* get_weaking_name(int x){
  return weakfunctions[x].name;
}