#ifndef __TIMER_H__
#define __TIMER_H__
#include "globals.h"

// Per-byte RX timeout = g_rx_timeout_ticks * ~100 ms (one Timer0_Start_Timeout).
// Default 5 ticks = ~500 ms, chosen to tolerate mid-APDU processing pauses
// from secure-element SAMs (HID Artemis SLE88, etc.) that may not emit T=0
// NULL/WTX bytes during AES/SCP02 operations. Set to 1 (~100 ms) to restore
// the original snappy "is the card alive" behaviour.
extern UINT8 g_rx_timeout_ticks;

void Timer0_Init();
void Timer0_Start_Timeout();
void Timer0_Stop_Timeout();
void Timer0_ResetTime();
void Timer0_UART_Recover();

#endif // __TIMER_H__