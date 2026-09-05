#ifndef __TIMER_H__
#define __TIMER_H__
#include "globals.h"

/*
 * Timer0 runs from Fsys/12; Timer3 supplies the UART baud clock.
 * With CKDIV = 0 the N76E003 sits at
 *
 *      Fsys = 16 MHz
 *
 * so Timer0 ticks every 0.75 us and a full 16 bit run only covers 49 ms. The
 * ISO/IEC 7816-3 block waiting time can be several seconds, so anything longer
 * than one timer run is counted in "slices" of 25 ms.
 */
/* PWM card clock: fixed 16 MHz CPU, 25 ms slices fit Timer0. */
#define SIM_FSYS_HZ     16000000UL
#define T0_SLICE_MS     25UL
#define T0_MS_TO_SLICES(ms) (((ms) + T0_SLICE_MS - 1UL) / T0_SLICE_MS)
#define T0_SLICE_TICKS  ((SIM_FSYS_HZ * T0_SLICE_MS + 11999UL) / 12000UL)
#define T0_SLICE_TH     ((UINT8)((65536UL - T0_SLICE_TICKS) >> 8))
#define T0_SLICE_TL     ((UINT8)(65536UL - T0_SLICE_TICKS))
#if T0_SLICE_TICKS > 65535UL
#error Timer0 slice exceeds its 16-bit counter
#endif

/* 1200 slices = 30 s. Nothing legitimate waits that long; this only keeps a
 * broken card from parking the module forever. */
#define T0_MAX_SLICES    T0_MS_TO_SLICES(30000UL)

void Timer0_Init(void);

/* arm one 25 ms slice - the caller polls TF0 itself */
void Timer0_Start_Slice(void);
void Timer0_Stop(void);

void Timer0_Delay_Slices(UINT16 slices);
void Timer0_Delay_Ticks(UINT32 ticks);

#endif // __TIMER_H__
