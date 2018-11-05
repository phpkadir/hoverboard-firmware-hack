#pragma once
typedef struct {int pwm; int weak;} RetValWeak;
typedef RetValWeak (*WeakingPtr)(int torque, uint period, uint cur_phase, int current);
typedef struct {WeakingPtr func; char name[8];} WeakStruct;
RetValWeak nullFuncWeak(int pwm, uint period, uint cur_phase, int current);
extern const WeakStruct weakfunctions[];