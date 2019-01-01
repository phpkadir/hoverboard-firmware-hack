#include "bldc.h"

volatile unsigned long buzzerStart;

void set_buzzerStart(unsigned long mainCnt){
    buzzerStart = mainCnt;
}