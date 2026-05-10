#include "timer.h"

// See timer.h for semantics. 5 * ~100 ms = ~500 ms total wait per byte.
UINT8 g_rx_timeout_ticks = 5;

void Timer0_Init() {
    clr_T0M;      //T0M=0, Timer0 Clock = Fsys/12
    TMOD |= 0x01; //Timer0 is 16-bit mode

    clr_TF0;
    clr_TR0;
}

void Timer0_Start_Timeout() {
    TL0 = 0xca; TH0 = 0x7d; // ~100 ms
    set_TR0;
}

void Timer0_Stop_Timeout() {
    clr_TF0;
    clr_TR0;
}

void Timer0_ResetTime() {
    TL0 = 0xe5; TH0 = 0xbe; // ~50 ms
    set_TR0;
    while(!TF0);
    clr_TF0;
    clr_TR0;
}

void Timer0_UART_Recover() {
    TL0 = 0xde; TH0 = 0xff; // ~100 us
    set_TR0;
    while(!TF0);
    clr_TF0;
    clr_TR0;
}