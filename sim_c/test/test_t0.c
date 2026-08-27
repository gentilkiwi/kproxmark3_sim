#include <stdio.h>
#include <string.h>
#include "globals.h"
#include "iso7816.h"
#include "uart.h"

void SEND_T0(void);
void SEND(void);
UINT16 recv_atr_to_pm3(UINT16 first_slices);
extern volatile UINT16 curr_sim_len;
extern volatile UINT16 to_pm3_len;
extern void t0_reset(void);
extern int t0_mode, t0_nulls, t0_outlen, t0_expect_in, t0_sw1, t0_sw2, t0_in_n;
extern int t0_recv_calls;
extern unsigned char t0_in[];
extern const unsigned char *t0_header(void);
extern void t0_push_raw(const unsigned char *b, int n);
extern int t0_rx_consumed(void);

static int fails;
#define CHECK(c) do { if(!(c)){ printf("  FAIL %d: %s\n", __LINE__, #c); fails++; } } while(0)

static UINT16 rlen(void){ return (UINT16)((to_pm3[0] << 8) | to_pm3[1]); }

static void setup(void){ t0_reset(); ISO7816_Reset_Params(); iso.active_proto = 0; }

int main(void) {
    printf("case 2 read, ACK for everything\n");
    setup();
    to_sim[0]=0x00; to_sim[1]=0xB0; to_sim[2]=0x00; to_sim[3]=0x00; to_sim[4]=0x08;
    curr_sim_len = 5; t0_outlen = 8;
    SEND_T0();
    CHECK(rlen() == 10);
    CHECK(to_pm3[2] == 0x50 && to_pm3[9] == 0x57);
    CHECK(to_pm3[10] == 0x90 && to_pm3[11] == 0x00);

    printf("case 3 write, ACK for everything\n");
    setup();
    to_sim[0]=0x00; to_sim[1]=0xD6; to_sim[2]=0x00; to_sim[3]=0x00; to_sim[4]=0x03;
    to_sim[5]=0xAA; to_sim[6]=0xBB; to_sim[7]=0xCC;
    curr_sim_len = 8; t0_expect_in = 3;
    SEND_T0();
    CHECK(rlen() == 2);
    CHECK(to_pm3[2] == 0x90 && to_pm3[3] == 0x00);
    CHECK(t0_in_n == 3 && t0_in[0] == 0xAA && t0_in[1] == 0xBB && t0_in[2] == 0xCC);

    printf("NULL bytes before the procedure byte must not shift the send index\n");
    setup();
    to_sim[0]=0x00; to_sim[1]=0xD6; to_sim[2]=0x00; to_sim[3]=0x00; to_sim[4]=0x03;
    to_sim[5]=0xAA; to_sim[6]=0xBB; to_sim[7]=0xCC;
    curr_sim_len = 8; t0_expect_in = 3; t0_nulls = 3;
    SEND_T0();
    CHECK(rlen() == 2);
    /* the old firmware resent P3 (0x03) here instead of the first data byte */
    CHECK(t0_in_n == 3 && t0_in[0] == 0xAA && t0_in[1] == 0xBB && t0_in[2] == 0xCC);

    printf("single byte ACK (~INS) mode\n");
    setup();
    to_sim[0]=0x00; to_sim[1]=0xD6; to_sim[2]=0x00; to_sim[3]=0x00; to_sim[4]=0x03;
    to_sim[5]=0x11; to_sim[6]=0x22; to_sim[7]=0x33;
    curr_sim_len = 8; t0_expect_in = 3; t0_mode = 1;
    SEND_T0();
    CHECK(rlen() == 2);
    CHECK(t0_in_n == 3 && t0_in[0] == 0x11 && t0_in[2] == 0x33);

    printf("card answers with SW straight away\n");
    setup();
    to_sim[0]=0x00; to_sim[1]=0xA4; to_sim[2]=0x04; to_sim[3]=0x00; to_sim[4]=0x00;
    curr_sim_len = 5; t0_mode = 2; t0_sw1 = 0x6A; t0_sw2 = 0x82;
    SEND_T0();
    CHECK(rlen() == 2);
    CHECK(to_pm3[2] == 0x6A && to_pm3[3] == 0x82);

    printf("card says nothing\n");
    setup();
    to_sim[0]=0x00; to_sim[1]=0xA4; to_sim[4]=0x00;
    curr_sim_len = 5; t0_mode = 3;
    SEND_T0();
    CHECK(rlen() == 0);

    printf("short command is rejected\n");
    setup(); curr_sim_len = 3;
    SEND_T0();
    CHECK(rlen() == 0);

    printf("long case 2 answer stays inside the buffer\n");
    setup();
    to_sim[0]=0x00; to_sim[1]=0xB0; to_sim[4]=0x00;
    curr_sim_len = 5; t0_outlen = 290;      /* more than the card should ever send */
    SEND_T0();
    CHECK(rlen() <= TRANSFER_MAX_DATA);
    CHECK(to_pm3_len <= TRANSFER_BUF_SIZE);

    /* ATR length comes from TS and T0, so the read ends on the last byte
     * instead of waiting out an idle gap. A sentinel must stay untouched. */
    {
        static const struct { const char *name; int len; unsigned char b[40]; } atrs[] = {
            { "Grace SAM, T=0 then T=1 then T=15, TCK", 15,
              {0x3B,0x95,0x96,0x80,0xB1,0xFE,0x55,0x1F,0xC7,0x47,0x72,0x61,0x63,0x65,0x13} },
            { "MasterCard, TB1 TC1 TD1, 10 historical, TCK", 19,
              {0x3B,0xEA,0x00,0x00,0x81,0x31,0xFE,0x45,
               0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x5A} },
            { "Fido SIM, TA1 TD1 TD2 TA3, 15 historical, TCK", 22,
              {0x3B,0x9F,0x96,0x80,0x1F,0xC7,0x80,0x31,0xE0,0x73,0xFE,0x21,
               0x1B,0x64,0x41,0x63,0x32,0x00,0x82,0x90,0x00,0xD1} },
            { "T=0 only, no TD1, no TCK", 7,
              {0x3B,0x05,0x11,0x22,0x33,0x44,0x55} },
            { "no interface bytes and no historical bytes", 2,
              {0x3B,0x00} },
        };
        static const unsigned char sentinel[1] = {0xA5};
        unsigned int t;

        for (t = 0; t < sizeof(atrs) / sizeof(atrs[0]); t++) {
            printf("ATR length: %s\n", atrs[t].name);
            setup();
            t0_push_raw(atrs[t].b, atrs[t].len);
            t0_push_raw(sentinel, 1);
            CHECK(recv_atr_to_pm3(4) == (UINT16)atrs[t].len);
            CHECK(to_pm3[PM3_CMD_HEADER_LEN] == atrs[t].b[0]);
            CHECK(to_pm3[PM3_CMD_HEADER_LEN + atrs[t].len - 1] == atrs[t].b[atrs[t].len - 1]);
            CHECK(t0_rx_consumed() == atrs[t].len);
        }

        printf("ATR that stops early is reported short, not overrun\n");
        setup();
        t0_push_raw(atrs[0].b, 4);
        CHECK(recv_atr_to_pm3(4) == 4);

        printf("no ATR at all\n");
        setup();
        CHECK(recv_atr_to_pm3(4) == 0);
    }

    /* A cut-off exchange must clear the line, or what the card was still
     * sending is read as the front of the next answer. The ordinary path must
     * not pay for that. */
    printf("a complete answer is not drained\n");
    setup();
    to_sim[0]=0x00; to_sim[1]=0xB0; to_sim[2]=0x00; to_sim[3]=0x00; to_sim[4]=0x04;
    curr_sim_len = 5; t0_outlen = 4;
    SEND_T0();
    CHECK(rlen() == 6);
    CHECK(to_pm3[6] == 0x90 && to_pm3[7] == 0x00);
    /* one call for the procedure byte, six for the answer, and no more: a
     * drain would show up here as an extra call that finds nothing */
    CHECK(t0_recv_calls == 7);

    printf("bytes left behind after a cut-off answer are cleared\n");
    setup();
    to_sim[0]=0x00; to_sim[1]=0xB0; to_sim[2]=0x00; to_sim[3]=0x00; to_sim[4]=0x08;
    curr_sim_len = 5; t0_outlen = 3;      /* fewer than Le, and no status word */
    t0_sw1 = 0x11; t0_sw2 = 0x22;         /* not a 6X/9X, so the answer is short */
    SEND_T0();
    /* whatever it handed back, the receiver must be empty afterwards */
    {
        unsigned char leftover;
        CHECK(UART_Recv(&leftover, 1) == 0);
    }

    printf("an unknown procedure byte clears the line\n");
    setup();
    to_sim[0]=0x00; to_sim[1]=0xB0; to_sim[4]=0x04;
    curr_sim_len = 5; t0_mode = 2; t0_sw1 = 0x33; t0_sw2 = 0x44;
    SEND_T0();
    CHECK(rlen() == 0 || rlen() == 2);
    {
        unsigned char leftover;
        CHECK(UART_Recv(&leftover, 1) == 0);
    }

    printf(fails ? "\n%d FAILURES\n" : "\nall T=0 checks passed\n", fails);
    return fails != 0;
}
