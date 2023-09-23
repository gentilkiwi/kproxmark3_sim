#ifndef __TIMER_H__
#define __TIMER_H__
#include "globals.h"

void Timer0_Init();
void Timer0_Start_Timeout();
void Timer0_Stop_Timeout();
void Timer0_ResetTime();
void Timer0_UART_Recover();

#endif // __TIMER_H__