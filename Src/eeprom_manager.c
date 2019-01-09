//ALPHA V0.1
// not implemented
#include <stdbool.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_flash.h"
#include "bldc.h"

uint64_t get_eeprom_version(){
    return 0;
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

void load_eeprom_v1(){

}

void save_eeprom_v1(){

}

const void (*eeprom_savers[])() = {
    save_eeprom_v1
};

const int eeprom_savers_lenght = 1;

void save_eeprom(){
    eeprom_savers[eeprom_savers_lenght-1]();
}

void load_eeprom(){
    switch(get_eeprom_version()){
        case 1:
            if(check_crc_v1()){
                load_eeprom_v1();
                return;
            } else {
                break;
            }
        case 0:
            if(check_crc_v0()){
                bldc_start_calibration();
                return;
            } else {
                break;
            }
        default:  // unknown version -> reset
            break;
    }
    reset_eeprom();
}

void reset_eeprom(){
    // clean eeprom set version 0
    set_eeprom_version(0);
}

__attribute__ ((section(".persistent_variables")))
volatile uint8_t persisten_mem[32*1024];