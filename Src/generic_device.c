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

#define BUFFERSIZE 128
#define VAL_CNT 2

uint32_t index[VAL_CNT] = {0,0};
int32_t buff_vals[VAL_CNT][BUFFERSIZE];
int32_t cur_buff_val_sum[VAL_CNT] = {0,0};

int32_t value_buffer(int32_t in,int val){
  cur_buff_val_sum[val] -= buff_vals[val][index[val]];
  cur_buff_val_sum[val] += (buff_vals[val][index[val]] = (in >> 16));
  index[val] = (index[val] + 1) % BUFFERSIZE;
  return (cur_buff_val_sum[val] / (BUFFERSIZE)) << 16;
}


//bobbycar
int clean_adc_full(uint32_t inval){
  int outval = (uint32_t)(inval >> 16) - ADC_MID;
  if(abs(outval) < (DEAD_ZONE / 2))
    return 0;
  else
    outval -= (DEAD_ZONE / 2) * SIGN(outval);
  if(abs(outval) > (ADC_MAX / 2 - ((DEAD_ZONE * 3) / 2)))
    return THROTTLE_MAX * SIGN(outval);
  return outval * THROTTLE_MAX / (ADC_MAX / 2 - ((DEAD_ZONE*3)/2));
}

int clean_adc_half(uint32_t inval){
  int outval = (uint32_t)(inval >> 16);
  if(abs(outval) > (ADC_MAX - ((DEAD_ZONE * 3) / 2)))
    return THROTTLE_MAX;
  return outval * THROTTLE_MAX / (ADC_MAX - ((DEAD_ZONE*3)/2));
}

void device_specific(){
	int tmp = -clean_adc_full(value_buffer(virtual_ival[0][0],0));
  int tmp2 = clean_adc_half(value_buffer(virtual_ival[0][1],1));
  int tmp3 = ((tmp*tmp2/THROTTLE_MAX)*(tmp*tmp2/THROTTLE_MAX) * SIGN(tmp*tmp2) / THROTTLE_MAX + (tmp*tmp2/THROTTLE_MAX)) / 2;
  if(tmp3 < 0) {
    tmp3 = tmp3 * THROTTLE_REVERSE_MAX / THROTTLE_MAX;
    set_buzzer(reverseSound);
  }

  else if(tmp3 >= 0)
    stop_buzzer();

  set_throttle(tmp3, tmp3);
}

void device_init(){
  HAL_Delay(50);
  //init_Display(4,0x3F);
  //divisor = 3;
  //weak = false;
  set_weaking(3);
  //PPM_Init();
  for(int i = 0; i < VAL_CNT ; i++)
    for(int j = 0; j < BUFFERSIZE;j++)
      buff_vals[i][j] = 0;
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