#ifndef __TIMER_H__
#define __TIMER_H__
#include "globals.h"

/*
 * Timer0 is the only free timer (Timer1 is the UART0 baud generator) and it
 * runs from Fsys/12.  With CKDIV = 2 the N76E003 sits at
 *
 *      Fsys = 16 MHz / (2 * CKDIV) = 4 MHz
 *
 * so Timer0 ticks every 3 us and a full 16 bit run only covers 196 ms.  The
 * ISO/IEC 7816-3 block waiting time can be several seconds, so anything longer
 * than one timer run is counted in "slices" of 50 ms.
 */
#define T0_SLICE_TICKS   16667U     /* 50 ms at 3 us / tick */
#define T0_SLICE_TH      0xBE
#define T0_SLICE_TL      0xE5

/* 600 slices = 30 s.  Nothing legitimate waits that long; this only keeps a
 * broken card from parking the module forever. */
#define T0_MAX_SLICES    600U

void Timer0_Init(void);

/* arm one 50 ms slice - the caller polls TF0 itself */
void Timer0_Start_Slice(void);
void Timer0_Stop(void);

void Timer0_Delay_Slices(UINT16 slices);
void Timer0_Delay_Ticks(UINT16 ticks);

#endif // __TIMER_H__
