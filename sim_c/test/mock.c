/* Host mock: the card side of the UART plus a small T=1 card model. */
#include <stdio.h>
#include <string.h>
#include "uart.h"
#include "timer.h"
#include "iso7816.h"

bit uart_parity_err;

static unsigned char tocard[4096]; static int tocard_n;
static unsigned char toterm[8192]; static int toterm_rd, toterm_wr;

void card_step(void);

void UART_Send(UINT8 c) { tocard[tocard_n++] = c; card_step(); }
int mock_rx_timeouts;           /* how many reads came back empty */

UINT8 UART_Recv(UINT8 *p, UINT16 slices) {
    (void)slices;
    if (toterm_rd < toterm_wr) { *p = toterm[toterm_rd++]; return 1; }
    *p = 0; mock_rx_timeouts++; return 0;
}
void UART_Drain(UINT16 s){ (void)s; toterm_rd = toterm_wr; uart_parity_err = 0; }
void UART_Set_Reload(UINT16 r){ (void)r; }
void UART_Set_TH1(UINT8 t){ (void)t; }
void UART_Set_FiDi(UINT8 t){ (void)t; }
UINT16 UART_Get_Reload(void){ return 23; }
UINT16 UART_Ticks_Per_Etu(void){ return 30; }
UINT16 UART_Etu_To_Slices(UINT32 e){ UINT32 s=(e*30UL)/16667UL+1; if(s>600)s=600; return (UINT16)s; }
void UART_Set_Guardtime(UINT8 n){ (void)n; }
void UART_Rx_Reset(void){}
void UART_Init(void){}
void Timer0_Init(void){}
void Timer0_Start_Slice(void){}
void Timer0_Stop(void){}
void Timer0_Delay_Slices(UINT16 s){ (void)s; }
void Timer0_Delay_Ticks(UINT16 t){ (void)t; }

/* ------------------------------------------------------------------ */
int  cd_ifsc        = 32;   /* what the card can send in one block   */
int  cd_wtx_once    = 0;
int  cd_corrupt_n   = 0;    /* corrupt the next N blocks it sends    */
int  cd_ifs_req     = 0;    /* ask the terminal to shrink IFSC first */
int  cd_ifs_req_val = 16;
int  cd_drop_n      = 0;    /* silently drop the next N blocks       */
int  cd_resp_len    = 4;

int  cd_ns, cd_expect_nr;
unsigned char cd_cmd[4096]; int cd_cmd_n;
static unsigned char cd_resp[4096]; static int cd_resp_len_cur, cd_resp_off;
static unsigned char cd_last[300]; static int cd_last_n;
int  cd_blocks_sent;

static void raw(const unsigned char *b, int n) { memcpy(toterm + toterm_wr, b, n); toterm_wr += n; }

static void emit(unsigned char pcb, const unsigned char *inf, int len) {
    unsigned char b[300]; unsigned char lrc = 0; int i, n = 0;
    b[n++] = 0x00; b[n++] = pcb; b[n++] = (unsigned char)len;
    for (i = 0; i < len; i++) b[n++] = inf[i];
    for (i = 0; i < n; i++) lrc ^= b[i];
    b[n++] = lrc;
    memcpy(cd_last, b, n); cd_last_n = n;
    cd_blocks_sent++;
    if (cd_drop_n > 0)    { cd_drop_n--;    return; }
    if (cd_corrupt_n > 0) { cd_corrupt_n--; b[n-1] ^= 0xFF; }
    raw(b, n);
}

static void resend_last(void) {
    unsigned char b[300]; memcpy(b, cd_last, cd_last_n);
    if (cd_drop_n > 0)    { cd_drop_n--; return; }
    if (cd_corrupt_n > 0) { cd_corrupt_n--; b[cd_last_n-1] ^= 0xFF; }
    raw(b, cd_last_n);
}

static void build_response(void) {
    int i;
    cd_resp_len_cur = cd_resp_len;
    for (i = 0; i < cd_resp_len_cur; i++) cd_resp[i] = (unsigned char)(0x30 + (i % 60));
    if (cd_resp_len_cur >= 2) { cd_resp[cd_resp_len_cur-2] = 0x90; cd_resp[cd_resp_len_cur-1] = 0x00; }
    cd_resp_off = 0;
}

static void send_next_resp_chunk(void) {
    int left = cd_resp_len_cur - cd_resp_off;
    int chunk = left > cd_ifsc ? cd_ifsc : left;
    int more = (chunk < left);
    emit((unsigned char)((cd_ns << 6) | (more ? 0x20 : 0)), cd_resp + cd_resp_off, chunk);
    cd_resp_off += chunk;
    cd_ns ^= 1;
}

void card_step(void) {
    unsigned char pcb, lrc = 0; int len, i;

    if (tocard_n < 4) return;
    len = tocard[2];
    if (tocard_n < 3 + len + 1) return;
    for (i = 0; i < 3 + len; i++) lrc ^= tocard[i];
    pcb = tocard[1];

    if (lrc != tocard[3 + len]) {          /* bad checksum from the terminal */
        tocard_n = 0;
        emit((unsigned char)(0x80 | (cd_expect_nr << 4) | 1), 0, 0);
        return;
    }

    if ((pcb & 0x80) == 0) {                                    /* I block */
        int ns = (pcb >> 6) & 1, more = (pcb >> 5) & 1;
        if (ns != cd_expect_nr) {
            /* a repeated I block means our last answer never arrived */
            tocard_n = 0;
            resend_last();
            return;
        }
        memcpy(cd_cmd + cd_cmd_n, tocard + 3, len); cd_cmd_n += len;
        tocard_n = 0;
        cd_expect_nr ^= 1;
        if (more) { emit((unsigned char)(0x80 | (cd_expect_nr << 4)), 0, 0); return; }
        if (cd_ifs_req)  { unsigned char v = (unsigned char)cd_ifs_req_val;
                           cd_ifs_req = 0; emit(0xC1, &v, 1); return; }
        build_response();
        if (cd_wtx_once) { unsigned char m = 3; cd_wtx_once = 0; emit(0xC3, &m, 1); return; }
        send_next_resp_chunk();
        return;
    }

    if ((pcb & 0xC0) == 0x80) {                                 /* R block */
        int err = pcb & 0x0F;
        tocard_n = 0;
        if (err) { resend_last(); return; }
        if (cd_resp_off < cd_resp_len_cur) { send_next_resp_chunk(); return; }
        resend_last();
        return;
    }

    /* S block */
    {
        int req = ((pcb & 0x20) == 0), c = pcb & 0x1F;
        unsigned char inf0 = (len >= 1) ? tocard[3] : 0;
        tocard_n = 0;
        if (req) {
            if (c == 1) { emit(0xE1, &inf0, 1); return; }        /* S(IFS) request  */
            if (c == 0) { cd_ns = 0; cd_expect_nr = 0; emit(0xE0, 0, 0); return; }
            if (c == 2) { emit(0xE2, 0, 0); return; }
            emit((unsigned char)(0x80 | (cd_expect_nr << 4) | 2), 0, 0);
            return;
        }
        /* responses from the terminal */
        if (c == 3) { send_next_resp_chunk(); return; }          /* S(WTX) response */
        if (c == 1) { build_response(); send_next_resp_chunk(); return; } /* S(IFS) response */
        return;
    }
}

void card_reset(void) {
    tocard_n = 0; toterm_rd = toterm_wr = 0;
    cd_ns = 0; cd_expect_nr = 0; cd_cmd_n = 0;
    cd_resp_len_cur = cd_resp_off = 0; cd_last_n = 0; cd_blocks_sent = 0;
    cd_ifsc = 32; cd_wtx_once = 0; cd_corrupt_n = 0;
    cd_ifs_req = 0; cd_drop_n = 0; cd_resp_len = 4;
    mock_rx_timeouts = 0;
}
