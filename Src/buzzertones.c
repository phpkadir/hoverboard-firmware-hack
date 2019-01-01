#include <stdbool.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"  // main code for stm32 controller
#include "defines.h"  // for the macros
#include "bldc.h"
#include "config.h"  // the config

volatile unsigned long buzzerStart;

void set_buzzerStart(unsigned long mainCnt){
    buzzerStart = mainCnt;
}
/*
void oldBuzzer(){  // buzzer for creating sounds
  if (int buzzerFreq != 0 && (get_mainCounter() / 5000) % (buzzerPattern + 1) == 0) {
    if (get_mainCounter() %int buzzerFreq == 0)
      HAL_GPIO_TogglePin(BUZZER_PORT, BUZZER_PIN);
  } else {
      HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, 0);
  }
}
*/

void lowBattery1(){
    int buzzerFreq = 5;
    int buzzerPattern = 8;
}
void lowBattery2(){
    int buzzerFreq = 5;
    int buzzerPattern = 1;
}
void lowBattery3(){
    int buzzerPattern = 0;
      for (int i = 0; i < 8; i++) {
        int buzzerFreq = i;
        HAL_Delay(100);
      }
}

void startUpSound(){
    unsigned long curBuzzTime = get_mainCounter() - buzzerStart;
    if(curBuzzTime < PWM_FREQ * 8 / 10)
        create_buzzer_wave(curBuzzTime, 8 - (curBuzzTime / (PWM_FREQ / 10))); // 1-8khz 1khz step
    else
        stop_buzzer();
}

void shutDownSound(){
    unsigned long curBuzzTime = get_mainCounter() - buzzerStart;
    if(curBuzzTime < PWM_FREQ * 8 / 10)
        create_buzzer_wave(curBuzzTime, curBuzzTime / (PWM_FREQ / 10));  // 8-1khz 1khz step
    else
        stop_buzzer();
}

void create_buzzer_wave(unsigned long hTime, int freq){
    if(hTime % freq)
        HAL_GPIO_TogglePin(BUZZER_PORT, BUZZER_PIN);
}

void buttonRelease(){
    unsigned long curBuzzTime = get_mainCounter() - buzzerStart;
    if(curBuzzTime < PWM_FREQ / 4)  // 1/4s peep
        create_buzzer_wave(curBuzzTime, 4); // 2 khz
    else
        stop_buzzer();
}

void resetSound(){
    unsigned long curBuzzTime = get_mainCounter() - buzzerStart;
    switch(curBuzzTime / (PWM_FREQ / 8)){
        case 0:
            create_buzzer_wave(curBuzzTime, 8);
        case 1:
            break;
        case 2:
            create_buzzer_wave(curBuzzTime, 4);
        case 3:
            break;
        default:
            create_buzzer_wave(curBuzzTime, 8);
    }
}