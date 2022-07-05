#ifndef _BALANCE_H_
#define _BALANCE_H_
#include "main.h"


extern float balance_K;
extern float dt;
extern float gy;
extern float pitch;
extern float angle_ax;	
extern float pitch_zero;
void Pitch_compute(void );
void Pitch_init(void);
#endif