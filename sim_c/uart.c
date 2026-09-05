#include "uart.h"
#include "timer.h"

volatile bit uart_parity_err;

#define UART_RX_MASK 63u
static volatile UINT8 idata uart_rx_buf[64];
static volatile UINT8 data uart_rx_head;
static volatile UINT8 data uart_rx_tail;

/* Bank 1 keeps reception independent of the main loop and I2C ISR. */
void UART_ISR(void) interrupt 4 using 1 {
    UINT8 data c;
    UINT8 data next;

    if (RI) {
        c = SBUF;                       /* refresh PSW.P for the parity compare */
        ACC = c;                    /* refresh PSW.P for the parity comparison */
        if (P) {
            if (!RB8) uart_parity_err = 1;
        } else {
            if (RB8) uart_parity_err = 1;
        }
        clr_RI;
        next = (UINT8)((uart_rx_head + 1u) & UART_RX_MASK);
        if (next == uart_rx_tail) {
            uart_parity_err = 1;     /* lost byte: reject the frame */
        } else {
            uart_rx_buf[uart_rx_head] = c;
            uart_rx_head = next;
        }
    }
}

static UINT32 uart_reload      = UART_RELOAD_DEFAULT;
static UINT32 uart_guard_ticks = 0;
static UINT16 uart_card_div    = 4;
static UINT16 uart_f           = 372;
static UINT8 uart_d            = 1;
static UINT8 uart_guard_n;

/* ISO/IEC 7816-3:2006, tables 7 and 8.  A 0 marks an RFU entry. */
static const UINT16 code fi_table[16] = {
     372,  372,  558,  744, 1116, 1488, 1860,    0,
       0,  512,  768, 1024, 1536, 2048,    0,    0
};

static const UINT8 code di_table[16] = {
       0,    1,    2,    4,    8,   16,   32,   64,
      12,   20,    0,    0,    0,    0,    0,    0
};

static void uart_apply_rate(void);
static UINT32 uart_divisor(UINT16 f, UINT8 d, UINT16 card_div);

static void uart_card_clock(UINT16 divider) {
    UINT16 period;
    if (divider == 1) {
        clr_PIO01;
        set_CLOEN;
        return;
    }
    period = (UINT16)(divider - 1u);
    PWMPH = (UINT8)(period >> 8);
    PWMPL = (UINT8)period;
    PWM1H = (UINT8)((divider / 2u) >> 8);
    PWM1L = (UINT8)(divider / 2u);
    set_LOAD;
    set_PWMRUN;
    while (LOAD);
    clr_CLOEN;
    set_PIO01;                          /* PWM1 on P1.1, never SDA/P1.4 */
}

void UART_Clock_Init(void) {
    CKDIV = 0;
    clr_CLOEN;
    P11_PushPull_Mode;
    clr_PWMCKS;                         /* PWM clock = Fsys */
    PWMCON0 = 0;
    PWMCON1 = 0;                        /* independent, edge-aligned, /1 */
    PIOCON0 = 0;
    clr_PIO11;
    set_CLRPWM;
    uart_card_div = 4;
    uart_card_clock(uart_card_div);
}

UINT8 UART_Set_Clock(UINT8 divider) {
    UINT16 card_div = divider ? (UINT16)(2u * divider) : 1u;
    if (uart_divisor(uart_f, uart_d, card_div) < 2UL) return 0;
    uart_card_clock(card_div);
    uart_card_div = card_div;
    uart_apply_rate();
    return 1;
}

void UART_Init(void) {

    ES = 0;
    uart_rx_head = 0;
    uart_rx_tail = 0;
    PS = 1;                         /* preempt the I2C ISR */

    /* SM0:SM1 = 11 -> mode 3, 9 bit UART clocked by Timer3, REN off for now.
     *
     * Careful: SFR_Macro.h calls SCON.7 "FE" and the old code reached mode 3
     * via set_FE.  SCON.7 only means "framing error" when PCON.SMOD0 is set,
     * which it never is here, so the bit really is SM0.  Writing SCON in one
     * go says what we mean. */
    SCON = 0xD0;

    clr_TR1;
    set_SMOD;                               /* double the baud rate */
    T3CON = 0x20;                          /* Timer3 baud source, no prescale */
    clr_ET3;

    UART_Set_FiDi(0x11);
    uart_guard_ticks = 0;
    uart_guard_n = 0;
    uart_parity_err  = 0;

    REN = 0;                                /* half duplex: listen only on demand */
}

static UINT32 uart_divisor(UINT16 f, UINT8 d, UINT16 card_div) {
    return ((UINT32)f * card_div + (UINT32)8u * d) / ((UINT32)16u * d);
}

static void uart_apply_rate(void) {
    UINT32 count = uart_divisor(uart_f, uart_d, uart_card_div);
    UINT8 prescale = 0;
    UINT16 reload;

    while (count > 65536UL) {
        count = (count + 1UL) / 2UL;
        prescale++;
    }
    uart_reload = count << prescale;
    reload = (UINT16)(0UL - count);
    clr_TR3;                            /* RH3/RL3 must be written stopped */
    T3CON = (UINT8)(0x20u | prescale);
    RH3 = (UINT8)(reload >> 8);
    RL3 = (UINT8)reload;
    set_TR3;
    UART_Set_Guardtime(uart_guard_n);
}

UINT8 UART_FiDi_Supported(UINT8 ta1) {
    UINT16 f = fi_table[ta1 >> 4];
    UINT8 d = di_table[ta1 & 0x0F];
    if (!f || !d) return 0;
    /* SMOD=1 forbids an overflow every system clock on Timer1 and Timer3. */
    return uart_divisor(f, d, uart_card_div) >= 2UL;
}

void UART_Set_FiDi(UINT8 ta1) {
    if (!UART_FiDi_Supported(ta1)) return;
    uart_f = fi_table[ta1 >> 4];
    uart_d = di_table[ta1 & 0x0F];
    uart_apply_rate();
}

void UART_Set_Reload(UINT16 reload) {
    /* Preserve the legacy Timer1 byte's card-relative baud rate. */
    if (reload < 1) reload = 1;
    if (reload > 256) reload = 256;
    if ((UINT32)reload * uart_card_div < 2UL) return;
    uart_f = (UINT16)(16u * reload);
    uart_d = 1;
    uart_apply_rate();
}

void UART_Set_TH1(UINT8 th1) {
    UART_Set_Reload((UINT16)(256u - (UINT16)th1));
}

UINT32 UART_Get_Reload(void) {
    return uart_reload;
}

UINT32 UART_Ticks_Per_Etu(void) {
    /* 1 etu = 16 * R / Fsys seconds, one Timer0 tick = 12 / Fsys seconds, so an
     * etu is 4 * R / 3 ticks. SIM_CLC changes the PWM divider and recomputes R
     * while Fsys remains fixed.
     *
     * Rounded up, not truncated: everything derived from this is a timeout, and
     * 4 * R / 3 is rarely a whole number - at R = 23 it is 30.67, and taking 30
     * made every waiting time 2 % shorter than the card is entitled to. */
    return ((uart_reload * 4UL) + 2UL) / 3UL;
}

UINT16 UART_Etu_To_Slices(UINT32 etu) {
    UINT32 per_etu = (UINT32)UART_Ticks_Per_Etu();
    UINT32 slices;

    if (per_etu == 0) {
        per_etu = 1;
    }
    if (per_etu < ((372UL * uart_card_div + 11UL) / 12UL)) {
        per_etu = (372UL * uart_card_div + 11UL) / 12UL;
    }
    if (etu >= ((UINT32)T0_MAX_SLICES * T0_SLICE_TICKS) / per_etu) {
        return T0_MAX_SLICES;
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
    uart_guard_n = n;
    if ((n == 0) || (n == 0xFF)) {
        uart_guard_ticks = 0;
        return;
    }

    ticks = (UINT32)n * (UINT32)UART_Ticks_Per_Etu();
    uart_guard_ticks = ticks;
}

void UART_Send(UINT8 c) {

    /* Half duplex: TX and RX share one wire, so stop listening before we drive
     * it or we receive our own echo. */
    UART_Rx_Reset();

    /* PSW.P is set when ACC holds an odd number of ones, which is exactly the
     * even parity bit ISO 7816-3 wants on the wire. */
    ACC = c;
    TB8 = P;

    clr_TI;
    SBUF = c;
    while (!TI);
    clr_TI;

    REN = 1;
    ES = 1;                         /* capture replies before returning */

    if (uart_guard_ticks) {
        Timer0_Delay_Ticks(uart_guard_ticks);
    }
}

UINT8 UART_Recv(UINT8 *pChar, UINT16 slices) {
    UINT8 data c;
    UINT8 data got = 0;

    *pChar = 0;

    /* Reception now queues characters in UART_ISR while callers do other work. */
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
    ES = 1;

    if (uart_rx_head == uart_rx_tail) {
        if (slices == 0) {
            slices = 1;
        }
        while (slices--) {
            Timer0_Start_Slice();
            while ((uart_rx_head == uart_rx_tail) && !TF0);
            Timer0_Stop();
            if (uart_rx_head != uart_rx_tail) {
                break;
            }
        }
    }

    if (uart_rx_head != uart_rx_tail) {
        c = uart_rx_buf[uart_rx_tail];
        uart_rx_tail = (UINT8)((uart_rx_tail + 1u) & UART_RX_MASK);
        *pChar = c;
        got = 1;
    }

    return got;
}

/* Drop anything the line may have picked up and start listening from scratch.
 * Every exchange that begins with a transmission gets this for free from
 * UART_Send; use it before a reception that does not, such as an ATR. */
void UART_Rx_Reset(void) {
    ES = 0;
    REN = 0;
    clr_RI;
    uart_rx_head = 0;
    uart_rx_tail = 0;
}

UINT16 UART_Recv_Burst(UINT8 xdata * data dst, UINT16 data want, UINT16 first_slices, UINT16 gap_slices) {

    UINT16 data got = 0;
    UINT16 data slices;
    UINT8 data c;

    REN = 1;
    ES = 1;

    while (got < want) {

        if (uart_rx_head == uart_rx_tail) {

            slices = (got == 0) ? first_slices : gap_slices;
            if (slices == 0) {
                slices = 1;
            }

            while (slices--) {
                Timer0_Start_Slice();
                while ((uart_rx_head == uart_rx_tail) && !TF0);
                Timer0_Stop();
                if (uart_rx_head != uart_rx_tail) {
                    break;
                }
            }

            if (uart_rx_head == uart_rx_tail) {
                break;                  /* card stopped sending */
            }
        }

        c = uart_rx_buf[uart_rx_tail];
        uart_rx_tail = (UINT8)((uart_rx_tail + 1u) & UART_RX_MASK);

        *dst++ = c;
        got++;
    }

    return got;
}

void UART_Drain(UINT16 slices) {
    UINT8 c;

    while (UART_Recv(&c, slices)) {
        ;
    }
    uart_parity_err = 0;
}
