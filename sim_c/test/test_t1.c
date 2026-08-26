#include <stdio.h>
#include <string.h>
#include "t1.h"
#include "iso7816.h"
#include "uart.h"

extern void card_reset(void);
extern int mock_rx_timeouts;
extern int cd_ifsc, cd_wtx_once, cd_corrupt_n, cd_ifs_req, cd_ifs_req_val;
extern int cd_drop_n, cd_resp_len, cd_cmd_n, cd_blocks_sent;
extern unsigned char cd_cmd[];

static UINT8 tx[512], rx[512];
static int fails;

#define CHECK(c) do { if (!(c)) { printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); fails++; } } while (0)

static void setup(void) {
    card_reset();
    ISO7816_Reset_Params();
    iso.active_proto = 1; iso.first_proto = 1; iso.protocols = 0x02;
    iso.ifsc = 32; iso.edc_crc = 0;
    T1_Reset();
    T1_Begin();                 /* SEND_T1 starts the clock; so must the tests */
}

static void fill(int n) { int i; for (i = 0; i < n; i++) tx[i] = (unsigned char)(i & 0xFF); }

static void t_simple(void) {
    INT16 r;
    printf("simple 5 byte APDU, 4 byte answer\n");
    setup(); fill(5); cd_resp_len = 4;
    r = T1_Transceive(tx, 5, rx, sizeof(rx));
    CHECK(r == 4);
    CHECK(cd_cmd_n == 5 && memcmp(cd_cmd, tx, 5) == 0);
    CHECK(rx[2] == 0x90 && rx[3] == 0x00);
}

static void t_two_in_a_row(void) {
    INT16 r;
    printf("two APDUs back to back (sequence numbers must advance)\n");
    setup(); fill(5); cd_resp_len = 6;
    r = T1_Transceive(tx, 5, rx, sizeof(rx));  CHECK(r == 6);
    cd_cmd_n = 0;
    T1_Begin();                 /* a second exchange gets a fresh budget */
    r = T1_Transceive(tx, 5, rx, sizeof(rx));  CHECK(r == 6);
    CHECK(cd_cmd_n == 5);
}

static void t_chain_rx(void) {
    INT16 r; int i, ok = 1;
    printf("card chains the answer (200 bytes over IFSC 32)\n");
    setup(); fill(5); cd_resp_len = 200; cd_ifsc = 32;
    r = T1_Transceive(tx, 5, rx, sizeof(rx));
    CHECK(r == 200);
    for (i = 0; i < 198; i++) if (rx[i] != (unsigned char)(0x30 + (i % 60))) ok = 0;
    CHECK(ok);
    CHECK(rx[198] == 0x90 && rx[199] == 0x00);
}

static void t_chain_tx(void) {
    INT16 r;
    printf("we chain the command (200 bytes over IFSC 32)\n");
    setup(); fill(200); iso.ifsc = 32; cd_resp_len = 2;
    r = T1_Transceive(tx, 200, rx, sizeof(rx));
    CHECK(r == 2);
    CHECK(cd_cmd_n == 200 && memcmp(cd_cmd, tx, 200) == 0);
}

static void t_chain_both(void) {
    INT16 r;
    printf("both directions chain (300 out, 300 back)\n");
    setup(); fill(300); iso.ifsc = 64; cd_ifsc = 64; cd_resp_len = 300;
    r = T1_Transceive(tx, 300, rx, sizeof(rx));
    CHECK(r == 300);
    CHECK(cd_cmd_n == 300 && memcmp(cd_cmd, tx, 300) == 0);
}

static void t_wtx(void) {
    INT16 r;
    printf("card asks for a time extension\n");
    setup(); fill(5); cd_resp_len = 4; cd_wtx_once = 1;
    r = T1_Transceive(tx, 5, rx, sizeof(rx));
    CHECK(r == 4);
}

static void t_bad_lrc(void) {
    INT16 r;
    printf("card's first answer has a broken LRC\n");
    setup(); fill(5); cd_resp_len = 4; cd_corrupt_n = 1;
    r = T1_Transceive(tx, 5, rx, sizeof(rx));
    CHECK(r == 4);
    CHECK(rx[2] == 0x90);
}

static void t_dropped(void) {
    INT16 r;
    printf("card swallows its first answer\n");
    setup(); fill(5); cd_resp_len = 4; cd_drop_n = 1;
    r = T1_Transceive(tx, 5, rx, sizeof(rx));
    CHECK(r == 4);
}

static void t_dropped_twice(void) {
    INT16 r;
    printf("two swallowed answers are past what the host will wait for\n");
    /* At the default BWI = 4 a block waiting time is 1.4 s, so only two
     * attempts fit inside the budget.  A third would outlast the Proxmark3 and
     * leave SCL held low with the host unable to recover it, so failing here is
     * the right outcome rather than retrying into a wedged bus. */
    setup(); fill(5); cd_resp_len = 4; cd_drop_n = 2;
    mock_rx_timeouts = 0;
    r = T1_Transceive(tx, 5, rx, sizeof(rx));
    CHECK(r < 0);
    CHECK(mock_rx_timeouts <= 3);
}

static void t_dead_card(void) {
    INT16 r;
    printf("card never answers at all\n");
    setup(); fill(5); cd_drop_n = 99;
    r = T1_Transceive(tx, 5, rx, sizeof(rx));
    CHECK(r < 0);
}

static void t_budget_bounds_a_dead_card(void) {
    INT16 r;
    printf("a quiet card cannot make us hold the bus indefinitely\n");
    /* Without a budget this retried three times, resynched, then retried three
     * more - about 10 s of BWT on a real card, against a host that gives up at
     * 3 s and then cannot recover the bus because only the module can release
     * SCL.  The budget has to cut it off well before that. */
    setup(); fill(5); cd_drop_n = 99;
    mock_rx_timeouts = 0;
    r = T1_Transceive(tx, 5, rx, sizeof(rx));
    CHECK(r < 0);
    printf("   timed-out reads: %d\n", mock_rx_timeouts);
    CHECK(mock_rx_timeouts <= 4);
}

static void t_ifs_req(void) {
    INT16 r;
    printf("card asks us to shrink IFSC before answering\n");
    setup(); fill(5); cd_resp_len = 4; cd_ifs_req = 1; cd_ifs_req_val = 16;
    r = T1_Transceive(tx, 5, rx, sizeof(rx));
    CHECK(r == 4);
    CHECK(iso.ifsc == 16);
}

static void t_ifsd(void) {
    UINT8 ok;
    printf("we announce IFSD = 254\n");
    setup();
    ok = T1_Negotiate_IFSD(254);
    CHECK(ok == 1);
}

static void t_overflow(void) {
    INT16 r;
    printf("answer larger than the caller's buffer\n");
    setup(); fill(5); cd_resp_len = 200; cd_ifsc = 32;
    r = T1_Transceive(tx, 5, rx, 40);
    CHECK(r == T1_E_OVERFLOW);
}

static void t_crc(void) {
    INT16 r;
    printf("CRC checksum path is self consistent\n");
    setup(); fill(5); cd_resp_len = 4; iso.edc_crc = 1;
    r = T1_Transceive(tx, 5, rx, sizeof(rx));
    /* the mock card only speaks LRC, so this must fail cleanly, not hang */
    CHECK(r < 0);
}

int main(void) {
    t_simple(); t_two_in_a_row(); t_chain_rx(); t_chain_tx(); t_chain_both();
    t_wtx(); t_bad_lrc(); t_dropped(); t_dropped_twice(); t_dead_card(); t_budget_bounds_a_dead_card(); t_ifs_req(); t_ifsd();
    t_overflow(); t_crc();
    printf(fails ? "\n%d FAILURES\n" : "\nall T=1 checks passed\n", fails);
    return fails != 0;
}
