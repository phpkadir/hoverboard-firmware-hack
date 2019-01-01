#include <stdbool.h>
#include <stdint.h>
#include "stm32f1xx_hal_flash.h"


__attribute__ ((section(".persistent_variables")))
volatile uint8_t persisten_mem[32*1024];