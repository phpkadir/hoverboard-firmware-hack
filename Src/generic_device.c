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

//rollbrett
int divisor = 2;
bool weak = false;
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
void device_specific(){
    int turn = clean_adc(virtual_ival[1][1]) / 4;
    int speed = clean_adc(virtual_ival[1][0])* 5 / 3 / divisor;
    set_throttle(speed + turn, speed - turn);
      // (adc_buffer.l_tx2-ADC_MID) / 2 + (adc_buffer.l_rx2-ADC_MID) / 2,
      // (adc_buffer.l_tx2-ADC_MID) / 2 - (adc_buffer.l_rx2-ADC_MID) / 2);
}

void device_init(){
  divisor = 2;
  weak = false;
  set_weaking(2);
}

void device_button(){
  if(divisor == 1 && weak){
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
  }
}

//for linking boost ups buildtime
const uint32_t lowBattery[] = {
  BATTERY_VOLTAGE2ADC12(BAT_LOW_DEAD),
  BATTERY_VOLTAGE2ADC12(BAT_LOW_LVL2),
  BATTERY_VOLTAGE2ADC12(BAT_LOW_LVL1)
};

const uint32_t lowBattery_length = 3;