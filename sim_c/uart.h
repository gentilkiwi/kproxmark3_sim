#ifndef __UART_H__
#define __UART_H__
#include "globals.h"

/*
 * UART0 talks to the card over the single half duplex I/O line.  It runs in
 * mode 3 (9 bit, Timer1 clocked) so that TB8 / RB8 can carry the ISO/IEC
 * 7816-3 parity bit, giving the required
 *
 *      1 start + 8 data + 1 parity + 1 stop = 11 etu
 *
 * character frame.  ISO uses even parity, which on the 8051 is exactly the PSW
 * parity flag of the data byte.
 *
 * Timer1 is the baud generator with SMOD = 1 and T1M = 1 (Timer1 counts Fsys):
 *
 *      baud = Fsys * 2 / (32 * R)      with R = 256 - TH1
 *
 * ISO wants baud = f_card * D / F and on this board f_card is Fsys (the CLO
 * output), so Fsys cancels out:
 *
 *      R = F / (16 * D)
 *
 * For the default F = 372, D = 1 that is R = 23.25.  The historical firmware
 * uses R = 24 (10417 baud, -3.1 % off the 10753 baud ideal), which came from a
 * "+1" in the old comment's formula; R = 23 gives 10870 baud, +1.1 % off.
 *
 * 24 is what ships, because it is the value this board has always run and the
 * signal path is not something to change while adding a feature.  Set this to
 * 23 if you want the more accurate divisor - it is the only line to touch.
 */
#define UART_RELOAD_DEFAULT   24

void UART_Init(void);

void   UART_Set_Reload(UINT16 reload);   /* reload = 256 - TH1, 2..256 */
void   UART_Set_TH1(UINT8 th1);          /* legacy I2C_DEVICE_CMD_SETBAUD payload */
void   UART_Set_FiDi(UINT8 ta1);         /* ISO 7816-3 TA1 (FI << 4 | DI) */
UINT16 UART_Get_Reload(void);

UINT16 UART_Ticks_Per_Etu(void);
UINT16 UART_Etu_To_Slices(UINT32 etu);

void UART_Set_Guardtime(UINT8 n);        /* ISO 7816-3 TC1 */

void  UART_Send(UINT8 c);
void  UART_Rx_Reset(void);
UINT8 UART_Recv(UINT8 *pChar, UINT16 slices);
void  UART_Drain(UINT16 slices);

/* Sticky, cleared by the caller.  Set whenever a character arrived with the
 * wrong parity bit.  T=1 turns this into an R block retransmission request. */
extern bit uart_parity_err;

#endif // __UART_H__
