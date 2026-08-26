#ifndef __UART_H__
#define __UART_H__
#include "globals.h"

/*
 * UART0 in mode 3 (9 bit, Timer1 clocked): TB8/RB8 carry the ISO 7816-3 parity
 * bit, giving the 1 start + 8 data + 1 parity + 1 stop frame. ISO uses even
 * parity, which is the 8051 PSW parity flag.
 *
 * Timer1 is the baud generator with SMOD = 1 and T1M = 1, so
 *      baud = Fsys * 2 / (32 * R),  R = 256 - TH1
 * and since f_card is Fsys here, R = F / (16 * D). F=372, D=1 gives R = 23.25.
 *
 * 23 (+1.1 %) not 24 (-3.1 %): at 24 the stop bit is sampled 10.84 bit times
 * in, leaving 0.16 bits before the next start bit. Enough for an ATR, whose
 * characters are spaced, but not for a T=1 block sent back to back.
 */
#define UART_RELOAD_DEFAULT   23

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
