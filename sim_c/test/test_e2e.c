/* SEND_T1 straight through SIM.C, against the T=1 card model. */
#include <stdio.h>
#include <string.h>
#include "globals.h"
#include "iso7816.h"
#include "t1.h"

void SEND_T1(void);
void SEND(void);
extern volatile UINT16 curr_sim_len;
extern volatile UINT16 to_pm3_len;
extern void card_reset(void);
extern int cd_resp_len, cd_ifsc, cd_cmd_n;
extern unsigned char cd_cmd[];
void I2C_Init(void){}
bit BIT_TMP;

static int fails;
#define CHECK(c) do { if(!(c)){ printf("  FAIL %d: %s\n", __LINE__, #c); fails++; } } while(0)
static UINT16 rlen(void){ return (UINT16)((to_pm3[0] << 8) | to_pm3[1]); }

static void setup(void) {
    card_reset(); ISO7816_Reset_Params();
    iso.active_proto = 1; iso.first_proto = 1; iso.protocols = ISO7816_PROTO_T1;
    T1_Reset();
}

int main(void) {
    int i;

    printf("SEND_T1: plain APDU in, plain APDU out\n");
    setup();
    to_sim[0]=0x00; to_sim[1]=0xA4; to_sim[2]=0x04; to_sim[3]=0x00; to_sim[4]=0x00;
    curr_sim_len = 5; cd_resp_len = 20;
    SEND_T1();
    CHECK(rlen() == 20);
    CHECK(to_pm3[2+18] == 0x90 && to_pm3[2+19] == 0x00);
    CHECK(cd_cmd_n == 5);
    CHECK(to_pm3_len == 22);

    printf("SEND_T1: answer that needs receive chaining\n");
    setup();
    curr_sim_len = 5; cd_resp_len = 250; cd_ifsc = 32;
    SEND_T1();
    CHECK(rlen() == 250);
    CHECK(to_pm3_len == 252);

    printf("SEND_T1: answer that would not fit is reported as empty, not as an overflow\n");
    setup();
    curr_sim_len = 5; cd_resp_len = 500; cd_ifsc = 64;
    SEND_T1();
    CHECK(rlen() == 0);

    printf("SEND_T1: maximum sized answer exactly fills the buffer\n");
    setup();
    curr_sim_len = 5; cd_resp_len = TRANSFER_MAX_DATA; cd_ifsc = 254;
    SEND_T1();
    CHECK(rlen() == TRANSFER_MAX_DATA);
    CHECK(to_pm3_len == TRANSFER_BUF_SIZE);

    printf("SEND_T1: long command chains out of to_sim\n");
    setup();
    for (i = 0; i < 260; i++) to_sim[i] = (UINT8)i;
    curr_sim_len = 260; iso.ifsc = 32; cd_resp_len = 2;
    SEND_T1();
    CHECK(rlen() == 2);
    CHECK(cd_cmd_n == 260);
    CHECK(memcmp(cd_cmd, to_sim, 260) == 0);

    printf("SEND_T1: empty command is rejected\n");
    setup(); curr_sim_len = 0;
    SEND_T1();
    CHECK(rlen() == 0);

    printf(fails ? "\n%d FAILURES\n" : "\nall end to end checks passed\n", fails);
    return fails != 0;
}
