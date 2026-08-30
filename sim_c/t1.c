#include "t1.h"
#include "iso7816.h"
#include "uart.h"
#include "timer.h"

/* NAD 0x00 means SAD = DAD = 0, which is what every card in the wild uses. */
#define T1_NAD              0x00

#define T1_MAX_RETRIES      3       /* ISO 7816-3 11.6.3.2 */
#define T1_MAX_RESYNCH      1
#define T1_BGT_ETU          22      /* block guard time, 11.4.3 */

#define T1_PCB_IS_I(p)      (((p) & 0x80) == 0x00)
#define T1_PCB_IS_R(p)      (((p) & 0xC0) == 0x80)
#define T1_PCB_IS_S(p)      (((p) & 0xC0) == 0xC0)

#define T1_ERR_NONE         0
#define T1_ERR_EDC          1
#define T1_ERR_OTHER        2

#define T1_S_RESYNCH        0
#define T1_S_IFS            1
#define T1_S_ABORT          2
#define T1_S_WTX            3
#define T1_S_VPPERR         4

#define T1_S_REQ            0xC0
#define T1_S_RSP            0xE0

#define T1_RX_OK            0
#define T1_RX_TIMEOUT       1
#define T1_RX_BADEDC        2
#define T1_RX_BADBLOCK      3

/* Sequence numbers belong to the link, not to a single APDU, so they survive
 * between commands and are only cleared by a card reset or a resynch. */
static UINT8 t1_ns;
static UINT8 t1_nr;
static UINT8 t1_ifsd;
static UINT8 t1_prepared;

/* header of the block we just received */
static UINT8 t1_r_pcb;
static UINT8 t1_r_len;
static UINT8 t1_r_inf0;

/* Keep the receive hot path in xdata so UART_Recv_Burst() can consume a
 * back-to-back T=1 frame without one C function call per character.  At
 * Fi=512/Di=16 (TA1=95) a character is only 88 us at the module's 4 MHz card
 * clock; the old byte-at-a-time state machine cannot reliably clear RI before
 * the following character arrives. */
static UINT8 xdata t1_rx_hdr[3];
static UINT8 xdata t1_rx_edc[2];

/* what to repeat when the card asks for a retransmission */
static UINT8  t1_last_pcb;
static UINT8  t1_last_is_i;
static UINT16 t1_last_off;
static UINT8  t1_last_len;
static UINT8  t1_last_inf;
static UINT8 xdata *t1_tx_base;

static UINT16 t1_edc;

/* What is left of the exchange's time budget, in 50 ms slices. */
static UINT16 t1_budget;

/* How long we may wait for the next thing, given what the budget has left. */
static UINT16 t1_take(UINT16 want) {
    if (want > t1_budget) {
        want = t1_budget;
    }
    return want;
}

/* A wait that timed out spent all of itself; one that did not spent almost
 * nothing, so only timeouts are charged. */
static void t1_spend(UINT16 used) {
    t1_budget = (t1_budget > used) ? (UINT16)(t1_budget - used) : 0;
}

/* ------------------------------------------------------------------------ */
/* checksum                                                                  */
/* ------------------------------------------------------------------------ */

static void t1_edc_init(void) {
    t1_edc = iso.edc_crc ? 0xFFFF : 0x0000;
}

static void t1_edc_update(UINT8 c) {
    UINT8 i;

    if (iso.edc_crc) {
        /* CRC per ISO/IEC 3309 (the HDLC frame check sequence): reflected
         * 0x1021 = 0x8408, seeded with ones, complemented at the end.
         *
         * NOTE: virtually every T=1 card in the field uses the LRC, so this
         * branch is the one to look at first if a CRC card misbehaves - the
         * standard's wording leaves room for the non reflected CCITT variant
         * (poly 0x1021, MSB first, no final complement) instead. */
        t1_edc ^= (UINT16)c;
        for (i = 0; i < 8; i++) {
            if (t1_edc & 0x0001) {
                t1_edc = (UINT16)((t1_edc >> 1) ^ 0x8408);
            } else {
                t1_edc = (UINT16)(t1_edc >> 1);
            }
        }
    } else {
        /* LRC, 11.3.2.3: xor of every byte of the block */
        t1_edc ^= (UINT16)c;
    }
}

/* ------------------------------------------------------------------------ */
/* block transmission                                                        */
/* ------------------------------------------------------------------------ */

static void t1_bgt(void) {
    /* At least 22 etu between the leading edge of the last character received
     * and the first one we send. */
    Timer0_Delay_Ticks((UINT16)((UINT16)T1_BGT_ETU * UART_Ticks_Per_Etu()));
}

static void t1_put(UINT8 c) {
    t1_edc_update(c);
    UART_Send(c);
}

static void t1_send_raw(UINT8 pcb, UINT8 xdata *inf, UINT8 len) {
    UINT8  i;
    UINT16 crc;

    t1_bgt();
    t1_edc_init();

    t1_put(T1_NAD);
    t1_put(pcb);
    t1_put(len);

    for (i = 0; i < len; i++) {
        t1_put(inf[i]);
    }

    if (iso.edc_crc) {
        crc = (UINT16)(t1_edc ^ 0xFFFF);
        UART_Send((UINT8)(crc >> 8));
        UART_Send((UINT8)(crc & 0xFF));
    } else {
        UART_Send((UINT8)(t1_edc & 0xFF));
    }
}

static void t1_send_i(UINT8 ns, UINT8 more, UINT16 off, UINT8 len) {
    t1_last_pcb  = (UINT8)((ns ? 0x40 : 0x00) | (more ? 0x20 : 0x00));
    t1_last_is_i = 1;
    t1_last_off  = off;
    t1_last_len  = len;
    t1_send_raw(t1_last_pcb, t1_tx_base + off, len);
}

static void t1_send_r(UINT8 nr, UINT8 err) {
    t1_last_pcb  = (UINT8)(0x80 | (nr ? 0x10 : 0x00) | (err & 0x0F));
    t1_last_is_i = 0;
    t1_last_len  = 0;
    t1_send_raw(t1_last_pcb, &t1_last_inf, 0);
}

static void t1_send_s(UINT8 pcb, UINT8 has_inf, UINT8 inf) {
    t1_last_pcb  = pcb;
    t1_last_is_i = 0;
    t1_last_inf  = inf;
    t1_last_len  = has_inf ? 1 : 0;
    t1_send_raw(pcb, &t1_last_inf, t1_last_len);
}

static void t1_retransmit(void) {
    if (t1_last_is_i) {
        t1_send_raw(t1_last_pcb, t1_tx_base + t1_last_off, t1_last_len);
    } else {
        t1_send_raw(t1_last_pcb, &t1_last_inf, t1_last_len);
    }
}

/* ------------------------------------------------------------------------ */
/* block reception                                                           */
/* ------------------------------------------------------------------------ */

/*
 * Reads exactly one block.  The first character gets first_slices (BWT, or an
 * extended BWT after S(WTX)), every following one only has to arrive within
 * CWT.  Because LEN tells us how long the block is we never sit out a timeout
 * at the end of a good block.
 *
 * The INF bytes land in dst, truncated at dstmax; the caller compares t1_r_len
 * against its own room to spot an overflow.  dst may be NULL when dstmax is 0.
 */
static UINT8 t1_recv(UINT8 xdata *dst, UINT16 dstmax, UINT16 first_slices) {

    UINT8  c;
    UINT8  i;
    UINT16 crc;

    uart_parity_err = 0;
    t1_edc_init();
    t1_r_pcb  = 0;
    t1_r_len  = 0;
    t1_r_inf0 = 0;

    if (UART_Recv_Burst(t1_rx_hdr, 3, first_slices, iso.cwt_slices) != 3) {
        return T1_RX_TIMEOUT;                    /* NAD */
    }
    t1_edc_update(t1_rx_hdr[0]);
    t1_r_pcb = t1_rx_hdr[1];
    t1_edc_update(t1_r_pcb);
    t1_r_len = t1_rx_hdr[2];
    t1_edc_update(t1_r_len);

    if (t1_r_len == 0xFF) {
        return T1_RX_BADBLOCK;                   /* 255 is RFU */
    }

    if ((dst != (UINT8 xdata *)0) && ((UINT16)t1_r_len <= dstmax)) {
        if (UART_Recv_Burst(dst, t1_r_len, iso.cwt_slices, iso.cwt_slices) != t1_r_len) {
            return T1_RX_TIMEOUT;
        }
        for (i = 0; i < t1_r_len; i++) {
            t1_edc_update(dst[i]);
        }
        if (t1_r_len) {
            t1_r_inf0 = dst[0];
        }
    } else {
        /* This is an S block with no destination, or an oversized I block.
         * It is exceptional; retain the bounded byte-wise drain so the line
         * is left clean and the caller can report the protocol error. */
        for (i = 0; i < t1_r_len; i++) {
            if (!UART_Recv(&c, iso.cwt_slices)) {
                return T1_RX_TIMEOUT;
            }
            t1_edc_update(c);
            if (i == 0) {
                t1_r_inf0 = c;
            }
            if (((UINT16)i < dstmax) && (dst != (UINT8 xdata *)0)) {
                dst[i] = c;
            }
        }
    }

    if (iso.edc_crc) {
        crc = (UINT16)(t1_edc ^ 0xFFFF);
        if (UART_Recv_Burst(t1_rx_edc, 2, iso.cwt_slices, iso.cwt_slices) != 2) {
            return T1_RX_TIMEOUT;
        }
        if ((t1_rx_edc[0] != (UINT8)(crc >> 8)) ||
            (t1_rx_edc[1] != (UINT8)(crc & 0xFF))) {
            return T1_RX_BADEDC;
        }
    } else {
        if (UART_Recv_Burst(t1_rx_edc, 1, iso.cwt_slices, iso.cwt_slices) != 1) {
            return T1_RX_TIMEOUT;
        }
        if (t1_rx_edc[0] != (UINT8)(t1_edc & 0xFF)) {
            return T1_RX_BADEDC;
        }
    }

    if (uart_parity_err) {
        return T1_RX_BADEDC;
    }
    return T1_RX_OK;
}

static UINT16 t1_wtx_slices(UINT8 mult) {
    UINT32 s;

    if (mult == 0) {
        mult = 1;
    }
    s = (UINT32)iso.bwt_slices * (UINT32)mult;
    if (s > (UINT32)T0_MAX_SLICES) {
        s = (UINT32)T0_MAX_SLICES;
    }
    return (UINT16)s;
}

/* ------------------------------------------------------------------------ */
/* link management                                                           */
/* ------------------------------------------------------------------------ */

void T1_Reset(void) {
    t1_ns        = 0;
    t1_nr        = 0;
    t1_ifsd      = 32;
    t1_prepared  = 0;
    t1_last_is_i = 0;
    t1_last_len  = 0;
}

UINT8 T1_Negotiate_IFSD(UINT8 ifsd) {
    UINT8 tries = 1;
    UINT16 give;

    if ((ifsd < 1) || (ifsd == 0xFF)) {
        return 0;
    }

    while (tries--) {

        give = t1_take(T1_HOUSEKEEPING_SLICES);
        if (give == 0) {
            return 0;
        }

        t1_send_s((UINT8)(T1_S_REQ | T1_S_IFS), 1, ifsd);

        if (t1_recv((UINT8 xdata *)0, 0, give) != T1_RX_OK) {
            t1_spend(give);
            continue;
        }
        if (T1_PCB_IS_S(t1_r_pcb) &&
            ((UINT8)(t1_r_pcb & 0x3F) == (UINT8)(0x20 | T1_S_IFS)) &&
            (t1_r_len == 1) && (t1_r_inf0 == ifsd)) {
            t1_ifsd = ifsd;
            return 1;
        }
    }
    return 0;
}

/* Start an exchange's clock.  Prepare and transceive share one budget, so the
 * housekeeping cannot eat the time the actual APDU needs. */
void T1_Begin(void) {
    t1_budget = T1_BUDGET_SLICES;
}

void T1_Prepare(void) {

    if (t1_prepared) {
        return;
    }
    t1_prepared = 1;

    /* If the ATR names T=0 first but also offers T=1, clause 9 says the card
     * will not listen to T=1 blocks until a PPS has switched it over. */
    /* A host-issued CMD_PPS may already have selected T=1.  Do not send a
     * second PPS here: PPS is only valid immediately after ATR and cards are
     * not required to accept another one once T=1 traffic has begun. */
    if ((iso.first_proto != 1) &&
        (iso.active_proto != 1) &&
        (iso.protocols & ISO7816_PROTO_T1)) {
        ISO7816_PPS(1, iso.ta1, 0);
    }

    /* The host asked for T=1, so that is what we speak from here on even if
     * the PPS above was refused - some cards accept T=1 regardless. */
    iso.active_proto = 1;

#if T1_IFSD_WANTED > 32
    T1_Negotiate_IFSD(T1_IFSD_WANTED);
#endif
}

/* ------------------------------------------------------------------------ */
/* one APDU in, one APDU out                                                 */
/* ------------------------------------------------------------------------ */

INT16 T1_Transceive(UINT8 xdata *tx, UINT16 txlen, UINT8 xdata *rx, UINT16 rxmax) {

    UINT16 tx_off  = 0;
    UINT16 rx_len  = 0;
    UINT16 wait;
    UINT8  retries = T1_MAX_RETRIES;
    UINT8  resyncs = T1_MAX_RESYNCH;
    UINT8  acked   = 0;
    UINT8  chunk;
    UINT8  more;
    UINT16 give;
    UINT8  r;
    UINT8  nr;
    UINT8  err;
    UINT8  scode;

    if ((txlen == 0) || (rxmax == 0)) {
        return T1_E_PARAM;
    }

    t1_tx_base = tx;

    chunk = (txlen > (UINT16)iso.ifsc) ? iso.ifsc : (UINT8)txlen;
    more  = ((UINT16)chunk < txlen) ? 1 : 0;
    t1_send_i(t1_ns, more, tx_off, chunk);

    wait = iso.bwt_slices;

    for (;;) {

        give = t1_take(wait);
        if (give == 0) {
            return T1_E_TIMEOUT;            /* out of time before the host is */
        }

        r    = t1_recv(rx + rx_len, (UINT16)(rxmax - rx_len), give);
        if (r == T1_RX_TIMEOUT) {
            t1_spend(give);
        }
        wait = iso.bwt_slices;              /* a WTX extension is good for one block */

        /* ---------------- transmission errors ---------------- */
        if (r != T1_RX_OK) {

            if (retries) {
                retries--;
                if (r == T1_RX_TIMEOUT) {
                    /* nothing came back at all - say it again, 11.6.3.2 */
                    t1_retransmit();
                } else {
                    /* something came back but it was damaged - ask for a repeat */
                    t1_send_r(t1_nr, (r == T1_RX_BADEDC) ? T1_ERR_EDC : T1_ERR_OTHER);
                }
                continue;
            }

            if (resyncs) {
                resyncs--;
                retries = T1_MAX_RETRIES;
                t1_send_s((UINT8)(T1_S_REQ | T1_S_RESYNCH), 0, 0);
                continue;
            }
            return T1_E_TIMEOUT;
        }

        /* ---------------- I block ---------------- */
        if (T1_PCB_IS_I(t1_r_pcb)) {

            if ((UINT8)((t1_r_pcb >> 6) & 0x01) != t1_nr) {
                /* the card repeated a block we already took - ask again for
                 * the one we are actually waiting for */
                if (retries) {
                    retries--;
                    t1_send_r(t1_nr, T1_ERR_OTHER);
                    continue;
                }
                return T1_E_PROTO;
            }

            if ((UINT16)t1_r_len > (UINT16)(rxmax - rx_len)) {
                return T1_E_OVERFLOW;
            }

            /* the first response block implicitly acknowledges our last one */
            if (!acked) {
                t1_ns ^= 1;
                acked  = 1;
            }

            rx_len  = (UINT16)(rx_len + t1_r_len);
            t1_nr  ^= 1;
            retries = T1_MAX_RETRIES;

            if (t1_r_pcb & 0x20) {          /* card is chaining, ask for more */
                t1_send_r(t1_nr, T1_ERR_NONE);
                continue;
            }
            return (INT16)rx_len;
        }

        /* ---------------- R block ---------------- */
        if (T1_PCB_IS_R(t1_r_pcb)) {

            nr  = (UINT8)((t1_r_pcb >> 4) & 0x01);
            err = (UINT8)(t1_r_pcb & 0x0F);

            if ((err == T1_ERR_NONE) && (nr != t1_ns) &&
                t1_last_is_i && ((UINT16)(tx_off + t1_last_len) < txlen)) {

                /* our chained I block was taken, push the next one */
                tx_off  = (UINT16)(tx_off + t1_last_len);
                t1_ns  ^= 1;
                retries = T1_MAX_RETRIES;

                chunk = ((UINT16)(txlen - tx_off) > (UINT16)iso.ifsc)
                            ? iso.ifsc : (UINT8)(txlen - tx_off);
                more  = ((UINT16)(tx_off + chunk) < txlen) ? 1 : 0;
                t1_send_i(t1_ns, more, tx_off, chunk);
                continue;
            }

            /* anything else means "I did not get that, send it again" */
            if (retries) {
                retries--;
                t1_retransmit();
                continue;
            }
            if (resyncs) {
                resyncs--;
                retries = T1_MAX_RETRIES;
                t1_send_s((UINT8)(T1_S_REQ | T1_S_RESYNCH), 0, 0);
                continue;
            }
            return T1_E_PROTO;
        }

        /* ---------------- S block ---------------- */
        scode = (UINT8)(t1_r_pcb & 0x1F);

        if ((t1_r_pcb & 0x20) == 0) {       /* a request from the card */

            switch (scode) {

                case T1_S_IFS:
                    /* the card is telling us how much it can take from now on */
                    if ((t1_r_len == 1) && (t1_r_inf0 != 0x00) && (t1_r_inf0 != 0xFF)) {
                        iso.ifsc = t1_r_inf0;
                    }
                    t1_send_s((UINT8)(T1_S_RSP | T1_S_IFS), 1, iso.ifsc);
                    break;

                case T1_S_WTX:
                    /* the card needs longer than BWT - grant it and echo back */
                    wait = t1_wtx_slices(t1_r_inf0);
                    t1_send_s((UINT8)(T1_S_RSP | T1_S_WTX), 1, t1_r_inf0);
                    break;

                case T1_S_ABORT:
                    t1_send_s((UINT8)(T1_S_RSP | T1_S_ABORT), 0, 0);
                    return T1_E_ABORT;

                case T1_S_VPPERR:
                    return T1_E_PROTO;

                case T1_S_RESYNCH:
                    /* only the interface device may ask for this */
                    t1_send_r(t1_nr, T1_ERR_OTHER);
                    break;

                default:
                    t1_send_r(t1_nr, T1_ERR_OTHER);
                    break;
            }
            continue;
        }

        /* a response to something we asked for */
        if (scode == T1_S_RESYNCH) {
            t1_ns = 0;
            t1_nr = 0;
            return T1_E_RESYNCH;            /* the caller may retry the APDU once */
        }
        if (scode == T1_S_IFS) {
            continue;
        }
        if (retries) {
            retries--;
            t1_send_r(t1_nr, T1_ERR_OTHER);
            continue;
        }
        return T1_E_PROTO;
    }
}
