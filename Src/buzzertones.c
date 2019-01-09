//FINAL V1.0
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

void lowBattery1(){
    unsigned long curBuzzTime = get_mainCounter() - buzzerStart;
    if(!((curBuzzTime / (PWM_FREQ / 3)) % 2))
        create_buzzer_wave(curBuzzTime, 4);
}

void lowBattery2(){
    unsigned long curBuzzTime = get_mainCounter() - buzzerStart;
    if(!((curBuzzTime / (PWM_FREQ / 4)) % 2))
        create_buzzer_wave(curBuzzTime, 3);
}

void lowBattery3(){
    unsigned long curBuzzTime = get_mainCounter() - buzzerStart;
    if(!((curBuzzTime / (PWM_FREQ / 6)) % 2))
        create_buzzer_wave(curBuzzTime, 2);
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
        create_buzzer_wave(curBuzzTime, (curBuzzTime / (PWM_FREQ / 10)) + 1);  // 8-1khz 1khz step
    else
        stop_buzzer();
}

void create_buzzer_wave(unsigned long hTime, int freq){
    if(hTime % freq)
        HAL_GPIO_TogglePin(BUZZER_PORT, BUZZER_PIN);
}

void reverseSound(){
    unsigned long curBuzzTime = get_mainCounter() - buzzerStart;
    if(!((curBuzzTime / (PWM_FREQ) % 2)))
        create_buzzer_wave(curBuzzTime, 5);
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

const void* lowBatTones[]={
    lowBattery3,
    lowBattery2,
    lowBattery1
};