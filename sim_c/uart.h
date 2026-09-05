#ifndef __UART_H__
#define __UART_H__
#include "globals.h"

/*
 * UART0 in mode 3 (9 bit, Timer3 clocked): TB8/RB8 carry the ISO 7816-3 parity
 * bit, giving the 1 start + 8 data + 1 parity + 1 stop frame. ISO uses even
 * parity, which is the 8051 PSW parity flag.
 *
 * Timer3 supplies baud = Fsys / (16 * R), including its prescaler in R.
 * PWM1 divides Fsys by four for the default card clock: R = F / (4 * D).
 * F=372,D=1 gives R=93; F=512,D=32 gives R=4, both exact.
 *
 * Legacy clocking used 23 (+1.1 %) not 24 (-3.1 %): at 24 the stop bit is sampled 10.84 bit times
 * in, leaving 0.16 bits before the next start bit. Enough for an ATR, whose
 * characters are spaced, but not for a T=1 block sent back to back.
 */
#define UART_RELOAD_DEFAULT   93

/* Ticks per etu at the default reload, 4 * R / 3 rounded up. Used as a floor
 * for the waiting times: see UART_Etu_To_Slices(). */
#define UART_TICKS_PER_ETU_DEFAULT  (((UART_RELOAD_DEFAULT * 4u) + 2u) / 3u)

void UART_Init(void);
void UART_Clock_Init(void);
UINT8 UART_Set_Clock(UINT8 divider);
UINT8 UART_FiDi_Supported(UINT8 ta1);

void   UART_Set_Reload(UINT16 reload);   /* legacy card-relative divisor, 1..256 */
void   UART_Set_TH1(UINT8 th1);          /* legacy I2C_DEVICE_CMD_SETBAUD payload */
void   UART_Set_FiDi(UINT8 ta1);         /* unsupported pairs leave the rate unchanged */
UINT32 UART_Get_Reload(void);

UINT32 UART_Ticks_Per_Etu(void);
UINT16 UART_Etu_To_Slices(UINT32 etu);

void UART_Set_Guardtime(UINT8 n);        /* ISO 7816-3 TC1 */

void  UART_Send(UINT8 c);
void  UART_Rx_Reset(void);
UINT8 UART_Recv(UINT8 *pChar, UINT16 slices);

/* Read up to `want` characters straight into xdata.
 *
 * UART_Recv() costs a function call, a generic pointer store - which on C51 is
 * a runtime helper, not a MOVX - and the caller's index arithmetic, for every
 * single byte. At the default etu there are 4464 instruction cycles per
 * character to absorb that; at Fi=512/Di=8 there are 768, and the line stops
 * working. This keeps the same framing and parity handling but pays the call
 * once per frame instead of once per byte, and writes through an xdata typed
 * pointer so the compiler emits MOVX @DPTR. T=1 uses this for each normal
 * receive frame, including its header and EDC.
 *
 * Returns how many arrived; short means the card stopped early. */
UINT16 UART_Recv_Burst(UINT8 xdata *dst, UINT16 want, UINT16 first_slices, UINT16 gap_slices);
void  UART_Drain(UINT16 slices);

/* Sticky, cleared by the caller.  Set whenever a character arrived with the
 * wrong parity bit.  T=1 turns this into an R block retransmission request. */
extern volatile bit uart_parity_err;

#endif // __UART_H__
