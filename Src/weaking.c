//ALPHA V0.1
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "stm32f1xx_hal.h"
#include "weaking.h"
#include "bldc.h"
#include "config.h"
#include "defines.h"

int last_speed = 0;

bool timing(uint8_t eagle/*0 to 255*/, int period, unsigned int cur_phase){
  if(eagle == 0)
    return false;  
  return cur_phase < (period * (UINT8_MAX - eagle) / UINT16_MAX);
}

RetValWeak longRange (int torque, int period, unsigned int cur_phase, int current){
  return (RetValWeak){ .pwm  = torque * PWM_MAX / THROTTLE_MAX, .weak = 0, .timing = false};
}
RetValWeak STVO6kmh(int torque, int period, unsigned int cur_phase, int current){
  if(period < 100)
    return (RetValWeak){ .pwm  = torque * PWM_MAX / THROTTLE_MAX, .weak = 0,  .timing = false};
  else
    return (RetValWeak){ .pwm  = torque * PWM_MAX / THROTTLE_MAX / 8, .weak = 0,  .timing = false};
}

RetValWeak nullFuncWeak(int torque, int period, unsigned int cur_phase, int current){
  return (RetValWeak){ .pwm  = 0, .weak = 0, .timing = false};
}
RetValWeak fastMode(int torque, int period, unsigned int cur_phase, int current){//todo same algorythmus than optweaking but less agressive and more efficient
  if(abs(torque) > (THROTTLE_MAX * 70 / 100) && abs(period) > 0)
    return (RetValWeak){
      .pwm  = PWM_MAX,
      .weak = (torque - (THROTTLE_MAX * 70 / 100)) * WEAKING_PWM_MAX /(THROTTLE_MAX * 30 / 100),
      .timing = timing(32 * torque / THROTTLE_MAX, period, cur_phase)
    };
  else
    return (RetValWeak){ .pwm  = torque * 100 * PWM_MAX / 70 / THROTTLE_MAX, .weak = 0, .timing = false};
}
RetValWeak optWeaking(int torque, int period, unsigned int cur_phase, int current){//todo
  return (RetValWeak){ .pwm  = torque, .weak = 0, .timing = timing(32 * torque / THROTTLE_MAX, period, cur_phase)};
}

int torgue2RPM(int torque){
  return 200000 / torque;
}

RetValWeak fixedRPM(int torque, int period, unsigned int cur_phase, int current){//todo
    int ideal_phase_period;
    if(torque !=0)
      ideal_phase_period = torgue2RPM(torque);
    else
      return (RetValWeak){ .pwm  = 0, .weak = 0, .timing = false};
    int current_phase;
    if(abs(period) < cur_phase)
      current_phase = SIGN(period) * cur_phase;
    else
      current_phase = period;
    if(ideal_phase_period>current_phase){
      //increase speed
      int temp_diff = ideal_phase_period - current_phase;
      last_speed += 50 * temp_diff / ideal_phase_period;
    }
    else{
      //degrease speed
      int temp_diff = ideal_phase_period - current_phase;
      last_speed -= last_speed * temp_diff / ideal_phase_period;
    }
  return (RetValWeak){ .pwm  = last_speed, .weak = 0, .timing = false};
}

const WeakStruct weakfunctions[] = {
  {nullFuncWeak," Off", 0},
  {STVO6kmh,"STVO", 5},
  {longRange,"LoRa", 25},
  {fastMode,"Fast",100},
  {optWeaking,"Race",200}
};
void set_weaking(int x){
  currentWeaking = weakfunctions[x].func;
  current_limit = weakfunctions[x].cur_limit;
}

const char* get_weaking_name(int x){
  return weakfunctions[x].name;
}

int get_currentWeaking(){
  int x = 0;
  while(weakfunctions[x].func != currentWeaking) x++;
  return x;
}