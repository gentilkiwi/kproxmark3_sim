#include "timer.h"

void Timer0_Init(void) {
    clr_T0M;                        /* Timer0 clock = Fsys/12 */
    TMOD = (UINT8)((TMOD & 0xF0) | 0x01);   /* Timer0 16 bit, leave Timer1 alone */
    clr_TF0;
    clr_TR0;
}

void Timer0_Start_Slice(void) {
    clr_TR0;
    clr_TF0;
    TL0 = T0_SLICE_TL;
    TH0 = T0_SLICE_TH;
    set_TR0;
}

void Timer0_Stop(void) {
    clr_TR0;
    clr_TF0;
}

void Timer0_Delay_Ticks(UINT16 ticks) {
    UINT16 reload;

    if (ticks < 4) {
        ticks = 4;                  /* below this the reload costs more than the wait */
    }
    reload = (UINT16)(0u - ticks);  /* 65536 - ticks */

    clr_TR0;
    clr_TF0;
    TL0 = (UINT8)(reload & 0xFF);
    TH0 = (UINT8)(reload >> 8);
    set_TR0;
    while (!TF0);
    clr_TR0;
    clr_TF0;
}

void Timer0_Delay_Slices(UINT16 slices) {
    while (slices--) {
        Timer0_Start_Slice();
        while (!TF0);
        Timer0_Stop();
    }
}
