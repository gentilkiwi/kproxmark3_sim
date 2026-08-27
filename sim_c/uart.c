#include "uart.h"
#include "timer.h"

bit uart_parity_err;

static UINT16 uart_reload      = UART_RELOAD_DEFAULT;
static UINT16 uart_guard_ticks = 0;

/* ISO/IEC 7816-3:2006, tables 7 and 8.  A 0 marks an RFU entry. */
static const UINT16 code fi_table[16] = {
     372,  372,  558,  744, 1116, 1488, 1860,    0,
       0,  512,  768, 1024, 1536, 2048,    0,    0
};

static const UINT8 code di_table[16] = {
       0,    1,    2,    4,    8,   16,   32,   64,
      12,   20,    0,    0,    0,    0,    0,    0
};

void UART_Init(void) {

    /* SM0:SM1 = 11 -> mode 3, 9 bit UART clocked by Timer1, REN off for now.
     *
     * Careful: SFR_Macro.h calls SCON.7 "FE" and the old code reached mode 3
     * via set_FE.  SCON.7 only means "framing error" when PCON.SMOD0 is set,
     * which it never is here, so the bit really is SM0.  Writing SCON in one
     * go says what we mean. */
    SCON = 0xD0;

    TMOD = (UINT8)((TMOD & 0x0F) | 0x20);   /* Timer1 8 bit auto reload */
    set_SMOD;                               /* double the baud rate */
    set_T1M;                                /* Timer1 counts Fsys, not Fsys/12 */
    clr_BRCK;                               /* UART0 baud source = Timer1 */

    UART_Set_Reload(UART_RELOAD_DEFAULT);
    uart_guard_ticks = 0;
    uart_parity_err  = 0;

    set_TR1;
    REN = 0;                                /* half duplex: listen only on demand */
}

void UART_Set_Reload(UINT16 reload) {
    /*
     * R = 1 is legal and is what an F = 512 / D = 32 card asks for: Timer1
     * overflows every Fsys tick, giving 250000 baud at Fsys = 4 MHz.  This used
     * to clamp at 2, which would have quietly run such a card at half rate.
     *
     * Whether the receive loop can keep up with a 44 us character at that speed
     * is another question - it is only reachable through a PPS that negotiates
     * TA1, so nothing takes that path by accident.
     */
    if (reload < 1) {
        reload = 1;
    }
    if (reload > 256) {
        reload = 256;
    }
    uart_reload = reload;
    TH1 = (UINT8)(256u - reload);           /* 256 wraps to 0, which is what the timer wants */
    TL1 = TH1;
}

void UART_Set_TH1(UINT8 th1) {
    /* Legacy I2C_DEVICE_CMD_SETBAUD, same as sim011.asm / sim013.asm: the host
     * writes Timer1's reload register straight through. */
    UART_Set_Reload((UINT16)(256u - (UINT16)th1));
}

void UART_Set_FiDi(UINT8 ta1) {
    UINT16 f;
    UINT8  d;

    f = fi_table[(ta1 >> 4) & 0x0F];
    d = di_table[ta1 & 0x0F];

    if ((f == 0) || (d == 0)) {
        return;                             /* RFU pair, keep the current rate */
    }

    /* R = round(F / (16 * D)) */
    UART_Set_Reload((UINT16)((f + (UINT16)(8u * d)) / (UINT16)(16u * d)));
}

UINT16 UART_Get_Reload(void) {
    return uart_reload;
}

UINT16 UART_Ticks_Per_Etu(void) {
    /* 1 etu = 16 * R / Fsys seconds, one Timer0 tick = 12 / Fsys seconds, so an
     * etu is 4 * R / 3 ticks whatever Fsys happens to be.  That is why SIM_CLC
     * (which changes CKDIV, hence Fsys and the card clock together) needs no
     * recalculation here.
     *
     * Rounded up, not truncated: everything derived from this is a timeout, and
     * 4 * R / 3 is rarely a whole number - at R = 23 it is 30.67, and taking 30
     * made every waiting time 2 % shorter than the card is entitled to. */
    return (UINT16)(((uart_reload * 4u) + 2u) / 3u);
}

UINT16 UART_Etu_To_Slices(UINT32 etu) {
    UINT32 per_etu = (UINT32)UART_Ticks_Per_Etu();
    UINT32 slices;

    if (per_etu == 0) {
        per_etu = 1;
    }
    slices = ((etu * per_etu) / (UINT32)T0_SLICE_TICKS) + 1UL;

    /*
     * A faster etu must not shorten how long we are prepared to wait for the
     * card to think.  ISO/IEC 7816-3 states the waiting times in etu, so
     * negotiating a quicker link scales them down - but a card's computation
     * time is wall clock and does not get faster because the wire did.  A SAM
     * that took 500 ms to answer still takes 500 ms after a PPS to Fi=512,
     * while WWT would have fallen from 950 ms to 350 ms and the exchange would
     * time out with no answer at all.
     *
     * Keep what the default etu would have allowed as a floor.  At the default
     * rate this changes nothing.
     */
    if (per_etu < (UINT32)UART_TICKS_PER_ETU_DEFAULT) {
        UINT32 floor_slices =
            ((etu * (UINT32)UART_TICKS_PER_ETU_DEFAULT) / (UINT32)T0_SLICE_TICKS) + 1UL;
        if (slices < floor_slices) {
            slices = floor_slices;
        }
    }

    if (slices > (UINT32)T0_MAX_SLICES) {
        slices = (UINT32)T0_MAX_SLICES;
    }
    return (UINT16)slices;
}

void UART_Set_Guardtime(UINT8 n) {
    UINT32 ticks;

    /* ISO/IEC 7816-3 TC1.  N = 255 asks for the minimum delay, 0 is the
     * default; both mean "no extra gap on top of the 11 etu the frame already
     * takes".  1..254 add N etu between characters we transmit. */
    if ((n == 0) || (n == 0xFF)) {
        uart_guard_ticks = 0;
        return;
    }

    ticks = (UINT32)n * (UINT32)UART_Ticks_Per_Etu();
    if (ticks > 60000UL) {
        ticks = 60000UL;
    }
    uart_guard_ticks = (UINT16)ticks;
}

void UART_Send(UINT8 c) {

    /* Half duplex: TX and RX share one wire, so stop listening before we drive
     * it or we receive our own echo. */
    REN = 0;
    clr_RI;

    /* PSW.P is set when ACC holds an odd number of ones, which is exactly the
     * even parity bit ISO 7816-3 wants on the wire. */
    ACC = c;
    TB8 = P;

    clr_TI;
    SBUF = c;
    while (!TI);
    clr_TI;

    if (uart_guard_ticks) {
        Timer0_Delay_Ticks(uart_guard_ticks);
    }
}

UINT8 UART_Recv(UINT8 *pChar, UINT16 slices) {
    UINT8 c;
    UINT8 got = 0;

    *pChar = 0;

    /*
     * REN stays on from here until the next UART_Send.  It used to be switched
     * off again at the end of every character, which loses the next one when a
     * card streams characters back to back - exactly what a T=1 block is.  The
     * gap between two calls is only about half a stop bit, and the checksum
     * update in between can easily be longer than that.
     *
     * The other half of that is not throwing away a character that already
     * arrived while we were busy, so RI is honoured before the timeout starts
     * rather than cleared.
     */
    REN = 1;

    if (!RI) {
        if (slices == 0) {
            slices = 1;
        }
        while (slices--) {
            Timer0_Start_Slice();
            while (!RI && !TF0);
            Timer0_Stop();
            if (RI) {
                break;
            }
        }
    }

    if (RI) {
        c   = SBUF;
        ACC = c;                    /* refresh PSW.P for the parity comparison */
        if (P) {
            if (!RB8) uart_parity_err = 1;
        } else {
            if (RB8)  uart_parity_err = 1;
        }
        clr_RI;
        *pChar = c;
        got = 1;
    }

    return got;
}

/* Drop anything the line may have picked up and start listening from scratch.
 * Every exchange that begins with a transmission gets this for free from
 * UART_Send; use it before a reception that does not, such as an ATR. */
void UART_Rx_Reset(void) {
    REN = 0;
    clr_RI;
}

void UART_Drain(UINT16 slices) {
    UINT8 c;

    while (UART_Recv(&c, slices)) {
        ;
    }
    uart_parity_err = 0;
}
