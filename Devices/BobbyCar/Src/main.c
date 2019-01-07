#include "bobbycar.h"
void device_specific(){
    int tmp_trottle[2]
    calc_torque_per_wheel(
      calc_torque(
        clean_adc(virtual_ival[0][0]),
        clean_adc(virtual_ival[0][1])),
      calc_steering_eagle(clean_adc(virtual_ival[1][0])),
      tmp_trottle);
    #ifdef BEEPS_BACKWARD
    if(tmp_trottle[0] + tmp_trottle[1] < 0){
      set_buzzer(reverseSound);
    }
    #endif
    set_throttle(tmp_trottle[0], tmp_trottle[1]);
}