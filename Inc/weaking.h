//BETA V0.1
#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct {int pwm; int weak; bool timing;} RetValWeak;
typedef RetValWeak (*WeakingPtr)(int torque, int period, unsigned int cur_phase, int current);
typedef struct {WeakingPtr func; const char* name; unsigned int cur_limit;} WeakStruct;
RetValWeak nullFuncWeak(int pwm, int period, unsigned int cur_phase, int current);
extern const WeakStruct weakfunctions[];
void set_weaking(int x);
const char* get_weaking_name(int x);
int get_currentWeaking();

