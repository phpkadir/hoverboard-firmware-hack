#include <math.h>
#include <stdlib.h>
#include "stm32f1xx_hal.h"  // main code for stm32 controller
#include "defines.h"  // for the macros
#include "setup.h"  // for access the functions form setup
#include "bldc.h"  // for the main control variables
#include "buzzertones.h"
#include "hd44780.h"  // for the display
#include "config.h"  // the config
#include "comms.h"
#include "control.h"

int ideal_phase_period = 0x8FFFFFFF;
int speed = 100;
uint8_t last_last_pos;
//rollbrett
int clean_adc(uint32_t inval){
  int outval = (uint32_t)(inval >> 16) - ADC_MID;
  if(abs(outval) < (DEAD_ZONE / 2))
    return 0;
  else
    outval -= (DEAD_ZONE / 2) * SIGN(outval);
  if(abs(outval) > (ADC_MAX / 2 - ((DEAD_ZONE * 3) / 2)))
    return THROTTLE_MAX * SIGN(outval);
  return outval * THROTTLE_MAX / (ADC_MAX / 2 - ((DEAD_ZONE*3)/2));
}

int clean_bobbycar(uint32_t inval){
  int outval = (uint32_t)(inval >> 16);
  if(abs(outval) > (ADC_MAX - ((DEAD_ZONE * 3) / 2)))
    return THROTTLE_MAX;
  return outval * THROTTLE_MAX / (ADC_MAX - ((DEAD_ZONE*3)/2));
}

//last_pos[0];
//timer[0];
//last_pos[0];
//phase_period[0];

void device_specific(){
  int current_phase;
  if(abs(phase_period[0]) < timer[0]){
    current_phase = SIGN(phase_period[0]) * timer[0];
  }
  else{
    current_phase = phase_period[0];
  }
  if(ideal_phase_period>phase_period){
    //increase speed
    speed*=2;
  }
  else{
    //degrease speed
    speed/=2;

  }
  int tmp = clean_adc(virtual_ival[1][1]);
    set_throttle(speed, 0);
      // (adc_buffer.l_tx2-ADC_MID) / 2 + (adc_buffer.l_rx2-ADC_MID) / 2,
      // (adc_buffer.l_tx2-ADC_MID) / 2 - (adc_buffer.l_rx2-ADC_MID) / 2);
}

void device_init(){
  last_last_pos = last_pos[0];
  ideal_phase_period = 5556;
  HAL_Delay(50);
  init_Display(4,0x3F);
  //divisor = 3;
  //weak = false;
  set_weaking(2);
  //PPM_Init();
}

void device_button(){
  /*if(divisor == 1 && weak){
    divisor = 3;
    weak = false;
    set_weaking(2);
  }
  else if(divisor == 3 && !weak){
    divisor = 2;
    weak = false;
    set_weaking(2);
  }
  else if(divisor == 1 && !weak){
    divisor = 1;
    weak = true;
    set_weaking(3);
  }
  else{
    divisor = 1;
    weak = false;
    set_weaking(2);
  }*/
}

//for linking boost ups buildtime
const uint32_t lowBattery[] = {
  BATTERY_VOLTAGE2ADC12(BAT_LOW_DEAD),
  BATTERY_VOLTAGE2ADC12(BAT_LOW_LVL2),
  BATTERY_VOLTAGE2ADC12(BAT_LOW_LVL1)
};

const uint32_t lowBattery_length = 3;