//ALPHA V0.1
// not implemented
#include <stdbool.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_flash.h"
#include "bldc.h"

#define EEPROM_VALUES_LENGHT (32*1024)
#define LATEST_EEPROM_VERSION 1

volatile uint8_t EEPROM_VALUES[EEPROM_VALUES_LENGHT] __attribute__ ((section(".persistent_variables")));

uint64_t get_eeprom_version(){
    return EEPROM_VALUES[EEPROM_VALUES_LENGHT-1-sizeof(uint64_t)];
}

bool check_crc_v1(){
    return true;
}

bool check_crc_v0(){
    //check if its resetted
    return true;
}

void set_eeprom_version(uint64_t version){

}

void load_eeprom_v0(){
    
}

void load_eeprom_v1(){
    
}


void save_eeprom_latest(){
    HAL_FLASH_Unlock();
    HAL_FLASH_Lock();
    
}

const void (*eeprom_loaders[])() = {
    load_eeprom_v0,
    load_eeprom_v1
};

const bool (*crc_checkers[])() = {
    check_crc_v0,
    check_crc_v1
};

void save_eeprom(){

}

void reset_eeprom(){
    set_eeprom_version(0);
}

void load_eeprom(){
    uint64_t version = get_eeprom_version();
    if(crc_checkers[version])
        eeprom_loaders[get_eeprom_version()];
    else
        reset_eeprom();
}