/*
 * Drives the slave interrupt handler through the transaction shapes the
 * Proxmark3 master in armsrc/i2c.c actually generates:
 *
 *   I2C_WriteCmd     START SLA+W cmd STOP
 *   I2C_WriteByte    START SLA+W cmd data STOP
 *   I2C_BufferWrite  START SLA+W cmd data... STOP
 *   I2C_BufferRead   START SLA+W cmd  rSTART SLA+R data... NACK STOP
 *
 * The I2C_BUS_ERR state is not exercised here: its recovery busy waits on the
 * hardware clearing STO, which a host build cannot model.
 */
#include <stdio.h>
#include <string.h>
#include "globals.h"
#include "i2c.h"
#include "iso7816.h"

void I2C_ISR(void);
extern volatile UINT16 curr_sim_len, to_pm3_len, curr_send_idx;
extern volatile UINT8  curr_cmd, have_cmd, rx_overflow;
extern volatile UINT8  pending_cmd;
void run_command(UINT8 cmd);
extern void card_reset(void);
extern int cd_resp_len;
void I2C_Init(void){}
bit BIT_TMP;

static int fails;
#define CHECK(c) do { if(!(c)){ printf("  FAIL %d: %s\n", __LINE__, #c); fails++; } } while(0)

/* every ISR exit must leave the slave listening and never bidding for master */
static void st(unsigned char s) {
    SI = 1;
    I2STAT = s;
    I2C_ISR();
    if (STA) { printf("  FAIL %d: STA left set after state 0x%02X\n", __LINE__, s); fails++; }
    if (STO) { printf("  FAIL %d: STO left set after state 0x%02X\n", __LINE__, s); fails++; }
    if (!AA) { printf("  FAIL %d: AA left clear after state 0x%02X\n", __LINE__, s); fails++; }
    if (SI)  { printf("  FAIL %d: SI  left set after state 0x%02X\n", __LINE__, s); fails++; }
}

/* the main loop: runs whatever the ISR queued, while SCL is held low */
static void pump(void) {
    if (pending_cmd) { run_command(pending_cmd); pending_cmd = 0; }
}

static void wr(unsigned char cmd, const unsigned char *d, int n) {
    int i;
    st(0x60);                                   /* own SLA+W, ACK */
    I2DAT = cmd; st(0x80);
    for (i = 0; i < n; i++) { I2DAT = d[i]; st(0x80); }
    st(0xA0);                                   /* STOP */
    pump();
}

/* master side of I2C_BufferRead, including its 2 byte length header parsing */
static int rd(unsigned char cmd, unsigned char *out, int max) {
    int got = 0, want = max, recv_len = 0;

    st(0x60);
    I2DAT = cmd; st(0x80);
    st(0xA0);                                   /* repeated START */
    pump();                                     /* SCL held low here */

    st(0xA8);                                   /* own SLA+R, ACK */
    out[got++] = I2DAT; want--;

    while (want > 0) {
        st(0xB8);                               /* previous byte was ACKed */
        out[got] = I2DAT;
        if (got == 1) {
            recv_len = (out[0] << 8) | out[1];
            if (want - 1 > recv_len) want = recv_len + 1;
        }
        got++; want--;
    }
    st(0xC0);                                   /* master NACKs the last byte */
    return (got >= 2) ? (int)((out[0] << 8) | out[1]) : -1;
}

static unsigned char buf[600];

int main(void) {
    int n, i;

    ISO7816_Reset_Params();
    card_reset();
    to_pm3_len = 0;

    printf("read before anything was queued yields a valid empty answer\n");
    n = rd(I2C_DEVICE_CMD_READ, buf, 270);
    CHECK(n == 0);
    CHECK(buf[0] == 0 && buf[1] == 0);

    printf("GETVERSION over a combined write/read transaction\n");
    n = rd(I2C_DEVICE_CMD_GETVERSION, buf, 4);
    CHECK(n == 2);
    CHECK(buf[2] == SIM_MODULE_VERS_HI && buf[3] == SIM_MODULE_VERS_LO);

    printf("command byte 0x00 is not mistaken for 'nothing latched yet'\n");
    {
        unsigned char d[3] = {0x11, 0x22, 0x33};
        wr(0x00, d, 3);
        CHECK(curr_sim_len == 3);
        CHECK(to_sim[0] == 0x11 && to_sim[2] == 0x33);
    }

    printf("SETBAUD payload lands in to_sim[0]\n");
    {
        unsigned char d[1] = {0xE9};
        wr(I2C_DEVICE_CMD_SETBAUD, d, 1);
        CHECK(curr_sim_len == 1 && to_sim[0] == 0xE9);
    }

    printf("SEND_T1 payload survives the transfer and the answer is framed\n");
    {
        unsigned char d[5] = {0x00, 0xA4, 0x04, 0x00, 0x00};
        card_reset();
        ISO7816_Reset_Params();
        iso.active_proto = 1; iso.first_proto = 1; iso.protocols = ISO7816_PROTO_T1;
        cd_resp_len = 12;
        wr(I2C_DEVICE_CMD_SEND_T1, d, 5);
        n = rd(I2C_DEVICE_CMD_READ, buf, 270);
        CHECK(n == 12);
        CHECK(buf[2 + 10] == 0x90 && buf[2 + 11] == 0x00);
    }

    printf("a write longer than to_sim is refused, not truncated into a command\n");
    {
        for (i = 0; i < 400; i++) buf[i] = (unsigned char)i;
        card_reset();
        wr(I2C_DEVICE_CMD_SEND_T1, buf, 400);
        CHECK(rx_overflow == 0);              /* cleared once handled */
        n = rd(I2C_DEVICE_CMD_READ, buf, 270);
        CHECK(n == 0);                        /* empty answer, no card traffic */
    }

    printf("still addressable right after an over long write\n");
    n = rd(I2C_DEVICE_CMD_GETVERSION, buf, 4);
    CHECK(n == 2);

    printf("arbitration lost states must not turn the slave into a master\n");
    st(0x68);   /* arb lost, own SLA+W received */
    st(0x78);   /* arb lost, general call received */
    st(0xB0);   /* arb lost, own SLA+R received  */
    CHECK(to_pm3_len >= PM3_CMD_HEADER_LEN);

    printf("a repeated START mid write drops the partial command\n");
    {
        st(0x60);
        I2DAT = I2C_DEVICE_CMD_SEND_T1; st(0x80);
        I2DAT = 0xAA; st(0x80);
        st(0xA0);                    /* dispatches with 1 byte */
        pending_cmd = 0;             /* do not run it, just check the bookkeeping */
        CHECK(curr_cmd == 0 && have_cmd == 0);
        st(0x60);                    /* new transfer starts clean */
        CHECK(curr_sim_len == 0);
    }

    printf("master reading past the end of the reply gets defined bytes\n");
    {
        to_pm3[0] = 0; to_pm3[1] = 2; to_pm3[2] = 0xAB; to_pm3[3] = 0xCD;
        to_pm3_len = 4;
        st(0xA8); CHECK(I2DAT == 0x00);
        st(0xB8); CHECK(I2DAT == 0x02);
        st(0xB8); CHECK(I2DAT == 0xAB);
        st(0xB8); CHECK(I2DAT == 0xCD);
        st(0xB8); CHECK(I2DAT == 0x00);   /* past the end */
        st(0xB8); CHECK(I2DAT == 0x00);
        st(0xC0);
    }

    printf(fails ? "\n%d FAILURES\n" : "\nall I2C slave checks passed\n", fails);
    return fails != 0;
}
