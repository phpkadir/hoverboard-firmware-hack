#pragma once
#include <stdbool.h>
typedef struct {int pwm; int weak;} RetValWeak;
typedef RetValWeak (*WeakingPtr)(int torque, uint period, uint cur_phase, int current);
typedef struct {WeakingPtr func; const char* name; uint cur_limit;} WeakStruct;
RetValWeak nullFuncWeak(int pwm, uint period, uint cur_phase, int current);
extern const WeakStruct weakfunctions[];

void set_weaking(int x);
bool calc_timing(int lst_phase, int cur_phase, int throttle);
const char* get_weaking_name(int x);
