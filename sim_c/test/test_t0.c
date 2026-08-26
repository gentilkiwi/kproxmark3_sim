#include <stdio.h>
#include <string.h>
#include "globals.h"
#include "iso7816.h"

void SEND_T0(void);
void SEND(void);
extern volatile UINT16 curr_sim_len;
extern volatile UINT16 to_pm3_len;
extern void t0_reset(void);
extern int t0_mode, t0_nulls, t0_outlen, t0_expect_in, t0_sw1, t0_sw2, t0_in_n;
extern unsigned char t0_in[];
extern const unsigned char *t0_header(void);

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

    printf(fails ? "\n%d FAILURES\n" : "\nall T=0 checks passed\n", fails);
    return fails != 0;
}
