#pragma once
#define WEAKING_PWM_MAX 450
typedef int (*WeakingPtr)(int pwm, uint period, uint cur_phase, int current);
typedef struct {WeakingPtr func; char name[8];} WeakStruct;
int nullFuncWeak(int pwm, uint period, uint cur_phase, int current);
extern const WeakStruct weakfunctions[];