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
 * used R = 24, which came from a "+1" in the old comment's formula and lands
 * 3.1 % slow; R = 23 is 1.1 % fast.  That difference decides whether a card can
 * be read back to back:
 *
 *   the receiver samples bit N at (N + 0.5) of its own bit time, so at the stop
 *   bit - index 10 - a 3.1 % slow clock samples 10.84 card bit times in.  The
 *   stop bit ends at 11.0, where the next character's start bit begins, so that
 *   leaves 0.16 bits of margin.  At 1.1 % fast it is 10.39 in, 0.61 bits of
 *   margin.
 *
 * An isolated character gets away with the tighter one because the line is idle
 * afterwards and a late sample still reads high - which is why an ATR, sent
 * with generous guard time, decodes perfectly at R = 24 while a T=1 block sent
 * at minimum spacing comes back corrupted.  The N76E003's internal RC is good
 * to a percent or two on top, which is enough to consume 0.16 bits entirely.
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
