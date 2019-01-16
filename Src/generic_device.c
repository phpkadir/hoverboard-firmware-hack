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
int clean_adc(uint32_t inval){
  int outval = inval >> 16 - ADC_MID;
  if(ABS(outval) < DEAD_ZONE/2)
    outval = 0;
  else
    outval -= DEAD_ZONE/2 * SIGN(outval);
}
void device_specific(){
    int turn = (adc_buffer.l_rx2 - ADC_MID) / 8;
    int speed = (adc_buffer.l_tx2 - ADC_MID) / (2 * divisor);

    if (ABS(turn) < 4) {
      turn = 0;
    } else {
      turn -= 4 * SIGN(turn);
    }

    if (ABS(speed) < 5) {
      speed = 0;
    }

    set_throttle(speed + turn, speed - turn);
      // (adc_buffer.l_tx2-ADC_MID) / 2 + (adc_buffer.l_rx2-ADC_MID) / 2,
      // (adc_buffer.l_tx2-ADC_MID) / 2 - (adc_buffer.l_rx2-ADC_MID) / 2);
}

void device_init(){
  set_weaking(2);
}

void device_button(){
  if(divisor == 1)
    divisor = 2;
  else
    divisor = 1;
}

//for linking boost ups buildtime
const uint32_t lowBattery[] = {
  BATTERY_VOLTAGE2ADC12(BAT_LOW_DEAD),
  BATTERY_VOLTAGE2ADC12(BAT_LOW_LVL2),
  BATTERY_VOLTAGE2ADC12(BAT_LOW_LVL1)
};

const uint32_t lowBattery_length = 3;