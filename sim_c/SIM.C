/*
 * Kiwi SIM module for Iceman
 *
 * ISO/IEC 7816-3 bridge between the Proxmark3 (which drives us as an I2C
 * slave) and a smart card (half duplex UART).  Speaks T=0 and T=1.
 *
 * The main loop does nothing but wait for the I2C ISR to hand it a command.
 * While a command runs we hold SCL low, which is the clock stretch the PM3
 * polls on to know the module is still busy talking to the card.
 *
 * The ISR hands over a command *number*, not a function pointer.  Keil counts
 * a function whose address is taken as being called from wherever the address
 * was taken, so a pointer parked by the ISR puts every handler into the
 * interrupt's call tree as well as main's.  Anything reachable from both then
 * trips "L15: MULTIPLE CALL TO SEGMENT" and loses its overlaid locals - which
 * on this part is not a warning to wave through, it is silent corruption of
 * whatever the linker decided to share the space with.
 */
#include "globals.h"
#include "timer.h"
#include "uart.h"
#include "i2c.h"
#include "iso7816.h"
#include "t1.h"

/* Buffer coming from the PM3, on its way to the card. */
UINT8 xdata to_sim[TRANSFER_BUF_SIZE];

/* Buffer going back to the PM3.  [0..1] is a big endian length header. */
UINT8 xdata to_pm3[TRANSFER_BUF_SIZE];

volatile UINT16 curr_sim_len  = 0;   /* bytes the PM3 gave us */
volatile UINT16 to_pm3_len    = 0;   /* bytes we have for the PM3, header included */
volatile UINT16 curr_send_idx = 0;   /* which of them we are clocking out */
volatile UINT8  curr_cmd      = 0;
volatile UINT8  have_cmd      = 0;   /* a command byte has been latched this transfer */
volatile UINT8  rx_overflow   = 0;   /* the PM3 wrote more than to_sim can hold */

/* Command the ISR has queued for the main loop, 0 when idle. */
volatile UINT8 pending_cmd = 0;

/* Not a wire command: queued when a transfer was refused before it could be
 * decoded, so the host still gets a well formed empty answer. */
#define CMD_REPLY_EMPTY   0xFF

static void run_command(UINT8 cmd);

/*
 * Inter character timeout once an answer has started.  The protocol waiting
 * times (WWT / BWT) only cover the gap before the first character; keeping the
 * follow up gap short is what makes an ordinary exchange finish quickly
 * instead of always sitting out a full WWT at the end.
 */
#define RX_IDLE_SLICES    2          /* 100 ms, what every firmware before this used */

/* The ATR must start within 40 000 clock cycles (~10 ms at 4 MHz) of RST going
 * high; be far more generous than that. */
#define ATR_WAIT_SLICES   4          /* 200 ms */

/* ISO/IEC 7816-3 7.2: at least 16 etu between characters travelling in
 * opposite directions.  The old code used a flat 100 us here. */
#define T0_TURNAROUND_ETU 16


void main(void) {

    Set_All_GPIO_Quasi_Mode;
    P10_PushPull_Mode;
    set_P10;                    /* card RST high */

    CKDIV = 2;                  /* Fsys = 16 / (2 * CKDIV) = 4 MHz, also the card clock */
    set_CLOEN;                  /* drive SIM_CLK from Fsys */

    UART_Init();
    Timer0_Init();
    ISO7816_Reset_Params();
    T1_Reset();
    I2C_Init();

    while (1) {

        if (pending_cmd) {

            /* SET CLOCK LINE - LOW, the PM3 waits on this */
            SCL = 0;

            run_command(pending_cmd);

            pending_cmd = 0;

            /* SET CLOCK LINE - HIGH */
            SCL = 1;
        }
    }
}

/* ------------------------------------------------------------------------ */
/* helpers                                                                   */
/* ------------------------------------------------------------------------ */

static void queue_pm3(UINT16 n) {
    if (n > TRANSFER_MAX_DATA) {
        n = TRANSFER_MAX_DATA;
    }
    to_pm3[0]  = (UINT8)(n >> 8);
    to_pm3[1]  = (UINT8)(n & 0xFF);
    to_pm3_len = (UINT16)(PM3_CMD_HEADER_LEN + n);
}

/*
 * Read from the card into to_pm3 until it goes quiet.
 *
 * The bounds check happens before the write, which the previous version got
 * the wrong way round: it tested `i < TRANSFER_BUF_SIZE` after storing at
 * to_pm3[2 + i], so a chatty card wrote two bytes past the end of the buffer.
 */
static UINT16 recv_to_pm3(UINT16 first_slices) {
    UINT16 n = 0;

    while (n < TRANSFER_MAX_DATA) {
        if (!UART_Recv(&to_pm3[PM3_CMD_HEADER_LEN + n],
                       (n == 0) ? first_slices : RX_IDLE_SLICES)) {
            break;
        }
        n++;
    }
    return n;
}

/*
 * Length driven read of a single T=1 block for the raw pass through path.  We
 * know from LEN exactly how many bytes are coming, so the answer is handed
 * back the moment the checksum arrives instead of after an idle timeout.
 */
static UINT16 recv_block_to_pm3(UINT16 first_slices) {
    UINT16 n = 0;
    UINT16 need;

    while (n < 3) {                             /* NAD PCB LEN */
        if (!UART_Recv(&to_pm3[PM3_CMD_HEADER_LEN + n],
                       (n == 0) ? first_slices : iso.cwt_slices)) {
            return n;
        }
        n++;
    }

    need = (UINT16)(3u + (UINT16)to_pm3[PM3_CMD_HEADER_LEN + 2]
                       + (iso.edc_crc ? 2u : 1u));
    if (need > TRANSFER_MAX_DATA) {
        need = TRANSFER_MAX_DATA;
    }

    while (n < need) {
        if (!UART_Recv(&to_pm3[PM3_CMD_HEADER_LEN + n], iso.cwt_slices)) {
            return n;
        }
        n++;
    }
    return n;
}

static void queue_sw_to_pm3(UINT8 sw1) {
    to_pm3[PM3_CMD_HEADER_LEN + 0] = sw1;

    /* SW2 must follow; report one byte rather than two if it never turns up
     * instead of shipping whatever was left in the buffer. */
    if (UART_Recv(&to_pm3[PM3_CMD_HEADER_LEN + 1], iso.wwt_slices)) {
        queue_pm3(2);
    } else {
        queue_pm3(1);
    }
}

/* ------------------------------------------------------------------------ */
/* commands                                                                  */
/* ------------------------------------------------------------------------ */

/*
 * Used when a transfer was refused before it ever reached a handler, so the
 * host still gets a well formed zero length answer instead of stale data or a
 * silent timeout.
 */
static void REPLY_EMPTY(void) {
    queue_pm3(0);
}

/* 0x06 - firmware version.  4.51 and up understand SEND_T1 and PPS. */
static void GETVERSION(void) {
    to_pm3[PM3_CMD_HEADER_LEN + 0] = SIM_MODULE_VERS_HI;
    to_pm3[PM3_CMD_HEADER_LEN + 1] = SIM_MODULE_VERS_LO;
    queue_pm3(2);
}

/* 0x01 - cold reset the card and collect the ATR. */
static void GENERATE_ATR(void) {
    UINT16 n;

    ISO7816_Reset_Params();
    T1_Reset();
    UART_Rx_Reset();                      /* start listening from a clean slate */

    clr_P10;                              /* RST low */
    Timer0_Delay_Slices(1);               /* 50 ms */
    set_P10;                              /* RST high */

    n = recv_to_pm3(ATR_WAIT_SLICES);

    if (n) {
        /* Everything downstream - waiting times, IFSC, checksum type, guard
         * time, which protocols exist - comes out of these bytes. */
        ISO7816_Parse_ATR(&to_pm3[PM3_CMD_HEADER_LEN], n);
    }
    queue_pm3(n);
}

/*
 * 0x02 - raw pass through.  The host owns the protocol, we only move bytes.
 *
 * The reply used to be collected with a flat 100 ms inter character timeout,
 * which is far too short for the first character of an answer: a default T=0
 * card is allowed WWT ~= 0.9 s and a default T=1 card BWT ~= 1.4 s of thinking
 * time.  The first character now gets the real protocol waiting time, and the
 * rest keeps the short gap so quick exchanges stay quick.
 */
static void SEND(void) {
    UINT16 i;
    UINT16 first;

    if (curr_sim_len == 0) {
        queue_pm3(0);
        return;
    }

    uart_parity_err = 0;

    for (i = 0; i < curr_sim_len; i++) {
        UART_Send(to_sim[i]);
    }

    if (iso.active_proto == 1) {
        first = iso.bwt_slices;
        queue_pm3(recv_block_to_pm3(first));
    } else {
        first = iso.wwt_slices;
        queue_pm3(recv_to_pm3(first));
    }
}

/*
 * 0x07 - the module runs the T=0 procedure byte exchange (ISO/IEC 7816-3
 * 10.3.3) so the host only has to hand over a plain APDU.
 *
 * Fixed here against the previous version:
 *   - a 0x60 NULL byte used to decrement the send index, so the next real ACK
 *     resent P3 instead of the first data byte;
 *   - a single byte ACK past the end of the command read past to_sim;
 *   - the response collector wrote two bytes past to_pm3 (see recv_to_pm3);
 *   - a missing SW2 was reported as if it had arrived.
 */
static void SEND_T0(void) {

    UINT8  procedure;
    UINT8  ins;
    UINT8  nibble;
    UINT8  guard = 0;
    UINT16 si;

    if (curr_sim_len < 5) {
        queue_pm3(0);
        return;
    }

    ins = to_sim[1];
    uart_parity_err = 0;

    /* the five byte command header */
    for (si = 0; si < 5; si++) {
        UART_Send(to_sim[si]);
    }

    for (;;) {

        if (++guard == 0) {                       /* runaway card, give up */
            queue_pm3(0);
            return;
        }

        if (!UART_Recv(&procedure, iso.wwt_slices)) {
            queue_pm3(0);
            return;
        }

        /* 0x60 is NULL: the card is only asking for more time.  Wait again,
         * and leave the send index exactly where it was. */
        if (procedure == 0x60) {
            continue;
        }

        Timer0_Delay_Ticks((UINT16)((UINT16)T0_TURNAROUND_ETU * UART_Ticks_Per_Etu()));

        /* SW1 is 0x6X (X != 0, handled above) or 0x9X and ends the command */
        nibble = (UINT8)(procedure >> 4);
        if ((nibble == 0x06) || (nibble == 0x09)) {
            queue_sw_to_pm3(procedure);
            return;
        }

        /* ACK for all remaining bytes */
        if (procedure == ins) {
            for (; si < curr_sim_len; si++) {
                UART_Send(to_sim[si]);
            }
            queue_pm3(recv_to_pm3(iso.wwt_slices));
            return;
        }

        /* ACK for exactly one more byte */
        if (procedure == (UINT8)(ins ^ 0xFF)) {
            if (si >= curr_sim_len) {
                /* nothing left to give - the card is answering, not asking */
                queue_pm3(recv_to_pm3(iso.wwt_slices));
                return;
            }
            UART_Send(to_sim[si]);
            si++;
            continue;
        }

        /* not a procedure byte we know */
        queue_pm3(0);
        return;
    }
}

/*
 * 0x08 - hand us a plain APDU, get a plain APDU back, T=1 handled here.
 * See t1.c for what that covers.
 */
static void SEND_T1(void) {
    INT16 n;

    if (curr_sim_len == 0) {
        queue_pm3(0);
        return;
    }

    /* One budget covers the whole thing - see T1_BUDGET_SLICES.  Holding the
     * I2C bus longer than the Proxmark3 waits for it wedges the bus rather
     * than merely failing the exchange. */
    T1_Begin();

    /* First call after an ATR: switch the card to T=1 if it did not come up
     * that way, and announce our IFSD.  Both are one shot. */
    T1_Prepare();

    n = T1_Transceive(to_sim, curr_sim_len,
                      &to_pm3[PM3_CMD_HEADER_LEN], TRANSFER_MAX_DATA);

    /* A resynch put both sides back to sequence number 0, so one clean retry
     * of the same APDU is worth having.  The budget is deliberately not reset:
     * the retry gets whatever time is left, never a second full allowance. */
    if (n == T1_E_RESYNCH) {
        n = T1_Transceive(to_sim, curr_sim_len,
                          &to_pm3[PM3_CMD_HEADER_LEN], TRANSFER_MAX_DATA);
    }

    queue_pm3((n > 0) ? (UINT16)n : 0);
}

/*
 * 0x09 - run a PPS exchange.
 *   to_sim[0]        protocol to select (0 or 1)
 *   to_sim[1]        optional TA1 (FI << 4 | DI) to negotiate as well
 * Answers with { ok, active protocol, TA1 in force }.
 */
static void PPS_EXCHANGE(void) {
    UINT8 ok;

    if (curr_sim_len < 1) {
        queue_pm3(0);
        return;
    }

    if (curr_sim_len > 1) {
        ok = ISO7816_PPS(to_sim[0], to_sim[1], 1);
    } else {
        ok = ISO7816_PPS(to_sim[0], iso.ta1, 0);
    }

    to_pm3[PM3_CMD_HEADER_LEN + 0] = ok;
    to_pm3[PM3_CMD_HEADER_LEN + 1] = iso.active_proto;
    to_pm3[PM3_CMD_HEADER_LEN + 2] = iso.ta1;
    queue_pm3(3);
}

/*
 * 0x04 - set the UART baud rate.  This was never implemented in the C
 * firmware; the payload byte is Timer1's reload register written straight
 * through, exactly as sim011.asm and sim013.asm did it.
 */
static void SETBAUD(void) {
    if (curr_sim_len >= 1) {
        UART_Set_TH1(to_sim[0]);
        ISO7816_Update_Timeouts();       /* the etu just changed */
    }
}

/*
 * 0x05 - set the card clock divider.  Also never implemented here before; the
 * payload byte goes straight into CKDIV like the assembly version did.
 *
 * Fsys is both the system clock and the card clock, so every waiting time we
 * keep in etu scales with it on its own - nothing else to recompute.
 */
static void SIM_CLC(void) {
    if (curr_sim_len >= 1) {
        CKDIV = to_sim[0];
    }
}

/* ------------------------------------------------------------------------ */
/* I2C slave, IRQ 6                                                          */
/* ------------------------------------------------------------------------ */

/*
 * Called from the main loop with SCL held low.  Direct calls, not a function
 * pointer: this keeps every handler in main's call tree only, so the linker
 * can overlay their locals against each other instead of refusing to (see the
 * note at the top of this file).
 */
static void run_command(UINT8 cmd) {

    switch (cmd) {
        case I2C_DEVICE_CMD_GETVERSION:   GETVERSION();   break;
        case I2C_DEVICE_CMD_GENERATE_ATR: GENERATE_ATR(); break;
        case I2C_DEVICE_CMD_SEND:         SEND();         break;
        case I2C_DEVICE_CMD_SEND_T0:      SEND_T0();      break;
        case I2C_DEVICE_CMD_SEND_T1:      SEND_T1();      break;
        case I2C_DEVICE_CMD_PPS:          PPS_EXCHANGE(); break;
        case I2C_DEVICE_CMD_SETBAUD:      SETBAUD();      break;
        case I2C_DEVICE_CMD_SIM_CLC:      SIM_CLC();      break;
        case CMD_REPLY_EMPTY:             REPLY_EMPTY();  break;
        default:                                          break;
    }
}

/*
 * I2C slave, IRQ 6
 *
 * The Proxmark3 is always the master and this module is always the slave, so
 * the interrupt only ever has to service the slave receive and slave transmit
 * states.  Two rules come out of that and are enforced on every exit:
 *
 *   - STA is never set.  The two "arbitration lost" states used to set it,
 *     which is the right answer for a device that was trying to be a master
 *     and lost - but this one never is.  Setting it makes the peripheral grab
 *     the bus and emit a START of its own as soon as the line goes idle, right
 *     in the middle of the Proxmark3's next transfer.
 *
 *   - AA is never left clear.  It used to be cleared to mark the last byte of
 *     a reply and to refuse an over long write.  AA also gates address
 *     recognition, so any path that ended while it was still clear left the
 *     module deaf to its own address until the next hardware reset.  The
 *     master decides how much it reads, and an over long write is now caught
 *     with a flag instead.
 *
 * This is what sim011.asm / sim013.asm have always done (clr STA, clr STO,
 * setb AA on every pass); the C rewrite is where it got lost.
 */
void I2C_ISR(void) interrupt 6
{
    switch (I2STAT) {

        case I2C_BUS_ERR: {
            /* Drop whatever half of a command we were holding, then let the
             * hardware push out a STOP to free the bus. */
            curr_cmd     = 0;
            have_cmd     = 0;
            curr_sim_len = 0;
            rx_overflow  = 0;

            set_AA;
            set_STO;
            clr_SI;
            while (STO);            /* hardware clears STO once the STOP is out */
            return;
        }

        /* ================ SLAVE RECEIVE ================
         * 15.3.3 - the PM3 is writing to us.
         */

        /* Own SLA+W (or general call) received, ACK transmitted.  0x68 / 0x78
         * are the arbitration lost flavours of the same thing; for a pure
         * slave they mean exactly the same as 0x60 / 0x70. */
        case I2C_GC_RECEIVE_ACK:
        case I2C_RECEIVE_ACK:
        case I2C_GC_RECEIVE_ABR_ERR:
        case I2C_RECEIVE_ABR_ERR: {
            curr_cmd     = 0;
            have_cmd     = 0;
            curr_sim_len = 0;
            rx_overflow  = 0;
            break;
        }

        /* Data byte received, ACK transmitted. */
        case I2C_GC_RECEIVE_DATA_OK:
        case I2C_RECEIVE_DATA_OK: {

            if (have_cmd == 0) {
                /* First byte after the address is the command.  A separate
                 * flag rather than "curr_cmd is still zero", so a command byte
                 * of 0x00 cannot be mistaken for "nothing latched yet". */
                curr_cmd = I2DAT;
                have_cmd = 1;
            } else if (curr_sim_len < TRANSFER_BUF_SIZE) {
                to_sim[curr_sim_len] = I2DAT;
                curr_sim_len++;
            } else {
                /* Keep acknowledging so the transfer ends cleanly, but refuse
                 * to act on a command we only have part of. */
                rx_overflow = 1;
            }
            break;
        }

        /* Data byte received, NACK transmitted.  With AA held set this should
         * not happen; if it does the byte stream is not trustworthy. */
        case I2C_GC_RECEIVE_DATA_ERR:
        case I2C_RECEIVE_DATA_ERR: {
            rx_overflow = 1;
            break;
        }

        /* A STOP or a repeated START arrived while we were still addressed.
         * Either way the command is complete, so hand it to the main loop.
         *
         * Note the main loop holds SCL low for as long as the handler runs, and
         * the PM3 only tolerates ~100 ms of that inside a transfer.  Anything
         * that can take longer (everything that talks to the card) must be
         * issued as write-then-STOP, followed by a separate READ; that is what
         * I2C_BufferWrite + sc_rx_bytes on the PM3 side already do. */
        case I2C_STOP_START: {

            if (rx_overflow) {
                pending_cmd = CMD_REPLY_EMPTY;
            } else {
                switch (curr_cmd) {
                    /* everything that needs the main loop to do some work */
                    case I2C_DEVICE_CMD_GETVERSION:
                    case I2C_DEVICE_CMD_GENERATE_ATR:
                    case I2C_DEVICE_CMD_SEND:
                    case I2C_DEVICE_CMD_SEND_T0:
                    case I2C_DEVICE_CMD_SEND_T1:
                    case I2C_DEVICE_CMD_PPS:
                    case I2C_DEVICE_CMD_SETBAUD:
                    case I2C_DEVICE_CMD_SIM_CLC: {
                        pending_cmd = curr_cmd;
                        break;
                    }
                    case I2C_DEVICE_CMD_READ:
                    default: {
                        /* READ needs nothing run - to_pm3 already holds the
                         * answer - and neither does a command we do not know.
                         * Leaving pending_cmd alone keeps the main loop from
                         * stretching SCL for no reason. */
                        break;
                    }
                }
            }

            curr_cmd    = 0;
            have_cmd    = 0;
            rx_overflow = 0;
            break;
        }

        /* ================ SLAVE TRANSMIT ================
         * 15.3.4 - the PM3 is reading from us.
         */

        /* Own SLA+R received, ACK transmitted.  0xB0 is the arbitration lost
         * flavour and needs the same handling. */
        case I2C_TRANSMIT_ADR_OK:
        case I2C_TRANSMIT_ABR_ERR: {

            /* Never clock out a half formed reply: if nothing has been queued
             * yet, present an empty one so the master reads a valid zero
             * length header instead of whatever the buffer happened to hold. */
            if (to_pm3_len < PM3_CMD_HEADER_LEN) {
                to_pm3[0]  = 0;
                to_pm3[1]  = 0;
                to_pm3_len = PM3_CMD_HEADER_LEN;
            }

            curr_send_idx = 0;
            I2DAT = to_pm3[curr_send_idx];
            curr_send_idx++;
            break;
        }

        /* Data byte transmitted, ACK received - the master wants another. */
        case I2C_TRANSMIT_DATA_OK: {
            if (curr_send_idx < to_pm3_len) {
                I2DAT = to_pm3[curr_send_idx];
                curr_send_idx++;
            } else {
                /* Read past the end of the reply.  The master clamps to the
                 * length header so this should not happen; send a defined byte
                 * rather than repeating the last one if it does. */
                I2DAT = 0x00;
            }
            break;
        }

        /* Data byte transmitted, NACK received - the master has what it wanted.
         * Last data byte transmitted, ACK received - same thing. */
        case I2C_TRANSMIT_DATA_ERR:
        case I2C_TRANSMIT_STOP: {
            break;
        }

        default: {
            /* 0xF8 (nothing to report) and anything undocumented. */
            break;
        }
    }

    /* Slave only, always listening.  See the note above the function. */
    clr_STA;
    clr_STO;
    set_AA;

    /* SI last, it is what releases SCL. */
    clr_SI;

} // end I2C_ISR
