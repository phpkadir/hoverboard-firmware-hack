#include "bobbycar.h"

int clean_adc(uint32_t inval){
  if((inval << 16)-DEAD_ZONE-ADC_MIN<0)
    return PWM_MIN;  // if ival in under deadzone
	else if((inval << 16)>ADC_MAX-2*DEAD_ZONE)
    return PWM_MAX;  // if ival in upper deadzone
  else
	  return (((inval << 16) - DEAD_ZONE - ADC_MIN) * (PWM_MAX - PWM_MIN)) / (ADC_MAX - ADC_MIN) + PWM_MIN;  // Map value linear to area in PWM area
}

int calc_torque(int throttle,int breaks){
  if(breaks == 0){  // drive forward
    return throttle;
}
  else if(breaks == PWM_MAX){  // drive backwards
    return -throttle;
  }
  else{
    return throttle-breaks;
  }
}
#define STEERING_EAGLE_FACTOR 0.01
float calc_steering_eagle(int inval){
  return (float)inval * STEERING_EAGLE_FACTOR;
}

inline void calc_torque_per_wheel(int throttle, float steering_eagle, int* torque){
  int back_wheel = WHEELBASE / tan(abs(steering_eagle));
  int radius_main = sqrt(pow(back_wheel, 2)+pow(WHEELBASE / 2 ,2));
#if !defined(STEERING)
  torque[0] = (back_wheel + WHEEL_WIDTH/2 * SIGN(steering_eagle)) * throttle / radius_main;
  torque[1] = (back_wheel - WHEEL_WIDTH/2 * SIGN(steering_eagle)) * throttle / radius_main;
#else
  #define wheel_bl (back_wheel + (WHEEL_WIDTH/2 * SIGN(steering_eagle) - STEERING_TO_WHEEL_DIST))
  #define wheel_br (back_wheel - (WHEEL_WIDTH/2 * SIGN(steering_eagle) - STEERING_TO_WHEEL_DIST))
  torque[0] = (sqrt(pow(wheel_bl, 2)+pow(WHEELBASE,2)) + STEERING_TO_WHEEL_DIST * SIGN(steering_eagle)) * throttle / radius_main;
  torque[1] = (sqrt(pow(wheel_br, 2)+pow(WHEELBASE,2)) - STEERING_TO_WHEEL_DIST * SIGN(steering_eagle)) * throttle / radius_main;
  #undef wheel_bl
  #undef wheel_br
#endif
}



/*int clean_adc(uint32_t inval){
  if((inval << 16)-DEAD_ZONE-ADC_MIN<0)
    return PWM_MIN;  // if ival in under deadzone
	else if((inval << 16)>ADC_MAX-2*DEAD_ZONE)
    return PWM_MAX;  // if ival in upper deadzone
  else
	  return (((inval << 16) - DEAD_ZONE - ADC_MIN) * (PWM_MAX - PWM_MIN)) / (ADC_MAX - ADC_MIN) + PWM_MIN;  // Map value linear to area in PWM area
}

#define WHEELBASE 2
#define WHEEL_WIDTH 1
#define STEERING_TO_WHEEL_DIST 1

inline void calc_torque_per_wheel(int throttle, float steering_eagle, int* torque){
  int back_wheel = WHEELBASE / tan(abs(steering_eagle));
  int radius_main = sqrt(pow(back_wheel, 2)+pow(WHEELBASE / 2 ,2));
#if !defined(STEERING)
  torque[0] = (back_wheel + WHEEL_WIDTH/2 * SIGN(steering_eagle)) * throttle / radius_main;
  torque[1] = (back_wheel - WHEEL_WIDTH/2 * SIGN(steering_eagle)) * throttle / radius_main;
#else
  #define wheel_bl (back_wheel + (WHEEL_WIDTH/2 * sign(steering_eagle) - STEERING_TO_WHEEL_DIST))
  #define wheel_br (back_wheel - (WHEEL_WIDTH/2 * sign(steering_eagle) - STEERING_TO_WHEEL_DIST))
  torque[0] = (sqrt(pow(wheel_bl, 2)+pow(WHEELBASE,2)) + STEERING_TO_WHEEL_DIST * sign(steering_eagle)) * throttle / radius_main;
  torque[1] = (sqrt(pow(wheel_br, 2)+pow(WHEELBASE,2)) - STEERING_TO_WHEEL_DIST * sign(steering_eagle)) * throttle / radius_main;
  #undef wheel_bl
  #undef wheel_br
#endif
}

int calc_torque(int throttle,int breaks){
  if(breaks == 0){  // drive forward
    return throttle;
}
  else if(breaks == PWM_MAX){  // drive backwards
    return -throttle;
  }
  else{
    return throttle-breaks;
  }
}
#define STEERING_EAGLE_FACTOR 0.01
float calc_steering_eagle(int inval){
  return (float)inval * STEERING_EAGLE_FACTOR;
}
*/