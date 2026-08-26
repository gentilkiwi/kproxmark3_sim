#include "iso7816.h"
#include "uart.h"
#include "timer.h"
#include "t1.h"      /* T1_PPS_SLICES */

iso7816_t xdata iso;

void ISO7816_Update_Timeouts(void) {

    /* T=0 work waiting time, ISO/IEC 7816-3 10.2:
     *      WWT = 960 * WI * (Fi / f) etu
     * with the 480 etu of slack the standard allows on top. */
    iso.wwt_slices = UART_Etu_To_Slices((960UL * (UINT32)iso.wi) + 480UL);

    /* T=1 character waiting time, 11.4.3:  CWT = 2 ^ CWI + 11 etu */
    iso.cwt_slices = UART_Etu_To_Slices((1UL << iso.cwi) + 11UL);

    /* T=1 block waiting time, 11.4.3:  BWT = 2 ^ BWI * 960 + 11 etu */
    iso.bwt_slices = UART_Etu_To_Slices(((1UL << iso.bwi) * 960UL) + 11UL);
}

void ISO7816_Reset_Params(void) {

    iso.valid        = 0;
    iso.protocols    = ISO7816_PROTO_T0;
    iso.first_proto  = 0;
    iso.active_proto = 0;
    iso.ta1          = 0x11;          /* F = 372, D = 1 */
    iso.guard_n      = 0;
    iso.wi           = 10;
    iso.ifsc         = 32;
    iso.bwi          = 4;
    iso.cwi          = 13;
    iso.edc_crc      = 0;

    /* A card reset also puts the card back on the default etu, so undo any
     * PPS or CMD_SETBAUD that happened since. */
    UART_Set_Reload(UART_RELOAD_DEFAULT);
    UART_Set_Guardtime(0);
    ISO7816_Update_Timeouts();
}

/*
 * ATR layout, ISO/IEC 7816-3 clause 8:
 *
 *   TS T0 [TA1 TB1 TC1 TD1] [TA2 TB2 TC2 TD2] ... [historical bytes] [TCK]
 *
 * T0 and every TDi carry Y in the high nibble (which of TA/TB/TC/TD follows)
 * and, for TDi, the protocol T in the low nibble.  Interface bytes in group 1
 * are global, group 2 is global plus the T=0 waiting time integer, and from
 * group 3 on they belong to the protocol named by the previous TDi.
 */
void ISO7816_Parse_ATR(UINT8 xdata *atr, UINT16 len) {

    UINT16 i;
    UINT8  y;
    UINT8  v;
    UINT8  group;
    UINT8  cur_t;
    UINT8  seen_td;

    ISO7816_Reset_Params();

    if (len < 2) {
        return;
    }

    /* TS is 0x3B (direct convention) or 0x3F (inverse).  The UART can only do
     * direct convention - an inverse card would need every byte bit reversed
     * and inverted, which the hardware will not do for us. */
    i = 1;

    v       = atr[i++];               /* T0 */
    y       = (UINT8)(v >> 4);
    group   = 1;
    cur_t   = 0;
    seen_td = 0;

    while (y && (i < len)) {

        if (y & 0x01) {               /* TA(group) */
            if (i >= len) break;
            v = atr[i++];
            if (group == 1) {
                iso.ta1 = v;
            } else if ((group >= 3) && (cur_t == 1)) {
                if ((v != 0x00) && (v != 0xFF)) {
                    iso.ifsc = v;     /* 0 and 255 are RFU */
                }
            }
        }

        if (y & 0x02) {               /* TB(group) */
            if (i >= len) break;
            v = atr[i++];
            if ((group >= 3) && (cur_t == 1)) {
                iso.bwi = (UINT8)(v >> 4);
                iso.cwi = (UINT8)(v & 0x0F);
                if (iso.bwi > 9) {
                    iso.bwi = 9;      /* > 9 is RFU */
                }
            }
        }

        if (y & 0x04) {               /* TC(group) */
            if (i >= len) break;
            v = atr[i++];
            if (group == 1) {
                iso.guard_n = v;
            } else if (group == 2) {
                iso.wi = (v == 0) ? 10 : v;
            } else if (cur_t == 1) {
                iso.edc_crc = (UINT8)(v & 0x01);
            }
        }

        if (y & 0x08) {               /* TD(group) */
            if (i >= len) break;
            v     = atr[i++];
            cur_t = (UINT8)(v & 0x0F);
            if (cur_t < 8) {
                iso.protocols |= (UINT8)(1u << cur_t);
            }
            if (!seen_td) {
                iso.first_proto = cur_t;
                iso.protocols   = (cur_t < 8) ? (UINT8)(1u << cur_t) : 0;
                seen_td         = 1;
            }
            y = (UINT8)(v >> 4);
        } else {
            y = 0;
        }

        group++;
    }

    if (!seen_td) {
        /* No TD1 at all means T=0 only, 8.2.3 */
        iso.protocols   = ISO7816_PROTO_T0;
        iso.first_proto = 0;
    }

    iso.valid        = 1;
    iso.active_proto = iso.first_proto;

    UART_Set_Guardtime(iso.guard_n);
    ISO7816_Update_Timeouts();
}

UINT8 ISO7816_PPS(UINT8 proto, UINT8 ta1, UINT8 use_ta1) {

    UINT8 buf[6];
    UINT8 n;
    UINT8 i;
    UINT8 pck;
    UINT8 pps0;

    proto = (UINT8)(proto & 0x0F);

    n        = 0;
    buf[n++] = 0xFF;                        /* PPSS */
    pps0     = proto;
    if (use_ta1) {
        pps0 |= 0x10;                       /* PPS1 present */
    }
    buf[n++] = pps0;
    if (use_ta1) {
        buf[n++] = ta1;                     /* PPS1 */
    }

    pck = 0;
    for (i = 0; i < n; i++) {
        pck ^= buf[i];
    }
    buf[n++] = pck;                         /* PCK */

    UART_Drain(1);
    uart_parity_err = 0;

    for (i = 0; i < n; i++) {
        UART_Send(buf[i]);
    }

    /* PPSS must come back verbatim.  A card answers a PPS out of its own state
     * rather than from a computation, so this gets a fixed short wait instead
     * of the full BWT - the module must not hold the I2C bus longer than the
     * host will wait for it.  See T1_BUDGET_SLICES in t1.h. */
    if (!UART_Recv(&buf[0], T1_PPS_SLICES) || (buf[0] != 0xFF)) {
        return 0;
    }
    if (!UART_Recv(&buf[1], iso.cwt_slices)) {
        return 0;
    }

    pps0 = buf[1];
    n    = 2;
    if (pps0 & 0x10) { if (!UART_Recv(&buf[n], iso.cwt_slices)) return 0; n++; }
    if (pps0 & 0x20) { if (!UART_Recv(&buf[n], iso.cwt_slices)) return 0; n++; }
    if (pps0 & 0x40) { if (!UART_Recv(&buf[n], iso.cwt_slices)) return 0; n++; }
    if (!UART_Recv(&buf[n], iso.cwt_slices)) {
        return 0;
    }

    pck = 0;
    for (i = 0; i <= n; i++) {
        pck ^= buf[i];
    }
    if (pck || uart_parity_err) {
        return 0;
    }

    /* The card confirms by echoing the protocol it accepted. */
    if ((UINT8)(pps0 & 0x0F) != proto) {
        return 0;
    }

    iso.active_proto = proto;

    /*
     * PPS1 only takes effect when the card echoed it back.  When it did not,
     * clause 9.3 says Fd = 372 and Dd = 1 apply from here on whatever TA1 of
     * the ATR proposed - so report and use the default rather than leaving
     * iso.ta1 holding a proposal that never came into force.
     */
    if ((pps0 & 0x10) && use_ta1) {
        iso.ta1 = buf[2];
        UART_Set_FiDi(buf[2]);
    } else {
        iso.ta1 = 0x11;
        UART_Set_Reload(UART_RELOAD_DEFAULT);
    }

    UART_Set_Guardtime(iso.guard_n);
    ISO7816_Update_Timeouts();

    return 1;
}
