//ALPHA V0.1
#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef bool (*TimingPtr)(int lst_phase, int cur_phase, int throttle);
bool no_timing(int lst_phase, int cur_phase, int throttle);