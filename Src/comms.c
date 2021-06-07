//ALPHA V0.1
#include <stdbool.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "stdio.h"
#include "string.h"
#include "bldc.h"
#include "control.h"
//UART_HandleTypeDef huart2;

volatile int32_t _buff_curlr[2];
volatile int32_t _buff_phase[2];
volatile uint32_t _buff_ival[2];
const int32_t *virtual_currentlr[] = {currentlr,_buff_curlr};
const int32_t *virutal_phase[] = {phase_period,_buff_phase};
#ifdef I2C_MASTER
const uint32_t *virtual_ival[] = {(uint32_t*)&(((uint16_t*)&adc_buffer)[6]),_buff_ival};  // magic code
#else
const uint32_t *virtual_ival[] = {_buff_ival,(uint32_t*)&(((uint16_t*)&adc_buffer)[6])};
#endif
const uint32_t **sync_vals[] = {virtual_ival,virtual_currentlr,virutal_phase};

#ifdef DEBUG_SERIAL_USART3
#define UART_DMA_CHANNEL DMA1_Channel2
#endif

#ifdef DEBUG_SERIAL_USART2
#define UART_DMA_CHANNEL DMA1_Channel7
#endif

volatile uint8_t uart_buf[100];
volatile int16_t ch_buf[8];
//volatile char char_buf[300];

void setScopeChannel(uint8_t ch, int16_t val) {
  ch_buf[ch] = val;
}

void consoleScope() {
  #ifdef DEBUG_SERIAL_SERVOTERM
    uart_buf[0] = 0xff;
    uart_buf[1] = CLAMP(ch_buf[0]+127, 0, 255);
    uart_buf[2] = CLAMP(ch_buf[1]+127, 0, 255);
    uart_buf[3] = CLAMP(ch_buf[2]+127, 0, 255);
    uart_buf[4] = CLAMP(ch_buf[3]+127, 0, 255);
    uart_buf[5] = CLAMP(ch_buf[4]+127, 0, 255);
    uart_buf[6] = CLAMP(ch_buf[5]+127, 0, 255);
    uart_buf[7] = CLAMP(ch_buf[6]+127, 0, 255);
    uart_buf[8] = CLAMP(ch_buf[7]+127, 0, 255);
    uart_buf[9] = '\n';

    if(UART_DMA_CHANNEL->CNDTR == 0) {
      UART_DMA_CHANNEL->CCR &= ~DMA_CCR_EN;
      UART_DMA_CHANNEL->CNDTR = 10;
      UART_DMA_CHANNEL->CMAR  = (uint32_t)uart_buf;
      UART_DMA_CHANNEL->CCR |= DMA_CCR_EN;
    }
  #endif

  #ifdef DEBUG_SERIAL_ASCII
    memset(uart_buf, 0, sizeof(uart_buf));
    sprintf(uart_buf, "%i;%i;%i;%i\n\r", ch_buf[0], ch_buf[1], ch_buf[2], ch_buf[3]);//, ch_buf[4], ch_buf[5], ch_buf[6], ch_buf[7]);

    if(UART_DMA_CHANNEL->CNDTR == 0) {
      UART_DMA_CHANNEL->CCR &= ~DMA_CCR_EN;
      UART_DMA_CHANNEL->CNDTR = strlen(uart_buf);
      UART_DMA_CHANNEL->CMAR  = (uint32_t)uart_buf;
      UART_DMA_CHANNEL->CCR |= DMA_CCR_EN;
    }
  #endif
}

void consoleLog(char *message)
{
    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)message, strlen(message));
}