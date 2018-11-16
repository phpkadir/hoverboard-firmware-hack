#pragma once
extern uint32_t timeout;

int clean_adc(int inval);
inline void calc_torque(int throttle,int breaks,int steering,int* torque);