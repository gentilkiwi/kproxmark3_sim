/* Host mock of a T=0 card, sitting behind the same UART API. */
#include <string.h>
#include "uart.h"
#include "timer.h"
#include "iso7816.h"

bit uart_parity_err;
bit BIT_TMP;

static unsigned char toterm[2048]; static int rd, wr;
static unsigned char hdr[5]; static int hdr_n;

int t0_mode;            /* 0 ack-all, 1 ack-one, 2 immediate SW, 3 no answer */
int t0_nulls;           /* how many 0x60 NULL bytes before the procedure byte */
int t0_outlen;          /* bytes the card returns for a case 2 command        */
int t0_expect_in;       /* bytes the card still wants for a case 3 command    */
int t0_sw1, t0_sw2;
unsigned char t0_in[300]; int t0_in_n;
int t0_single_left;
int t0_recv_calls;      /* UART_Recv calls: a drain shows up as an extra one */

static void push(unsigned char c){ toterm[wr++] = c; }
static void push_sw(void){ push((unsigned char)t0_sw1); push((unsigned char)t0_sw2); }
static void push_data(int n){ int i; for (i=0;i<n;i++) push((unsigned char)(0x50+i)); }

void UART_Send(UINT8 c) {
    int i;
    if (hdr_n < 5) {
        hdr[hdr_n++] = c;
        if (hdr_n < 5) return;
        for (i = 0; i < t0_nulls; i++) push(0x60);
        switch (t0_mode) {
            case 2: push((unsigned char)t0_sw1); push((unsigned char)t0_sw2); return;
            case 3: return;                                   /* card stays mute */
            case 1: t0_single_left = t0_expect_in;
                    push((unsigned char)(hdr[1] ^ 0xFF)); return;
            default:
                push(hdr[1]);                                 /* ACK for everything */
                if (t0_expect_in == 0) { push_data(t0_outlen); push_sw(); }
                return;
        }
    }
    /* data phase */
    t0_in[t0_in_n++] = c;
    if (t0_mode == 1) {
        if (--t0_single_left > 0) { push((unsigned char)(hdr[1] ^ 0xFF)); }
        else { push_data(t0_outlen); push_sw(); }
        return;
    }
    if (t0_in_n >= t0_expect_in) { push_data(t0_outlen); push_sw(); }
}

UINT8 UART_Recv(UINT8 *p, UINT16 s) {
    (void)s;
    t0_recv_calls++;
    if (rd < wr) { *p = toterm[rd++]; return 1; }
    *p = 0; return 0;
}
void UART_Drain(UINT16 s){ (void)s; rd = wr; }
void UART_Set_Reload(UINT16 r){ (void)r; }
void UART_Set_TH1(UINT8 t){ (void)t; }
void UART_Set_FiDi(UINT8 t){ (void)t; }
UINT16 UART_Get_Reload(void){ return 23; }
UINT16 UART_Ticks_Per_Etu(void){ return 30; }
UINT16 UART_Etu_To_Slices(UINT32 e){ UINT32 x=(e*30UL)/16667UL+1; if(x>600)x=600; return (UINT16)x; }
void UART_Set_Guardtime(UINT8 n){ (void)n; }
void UART_Rx_Reset(void){}
void UART_Init(void){}
void Timer0_Init(void){}
void Timer0_Start_Slice(void){}
void Timer0_Stop(void){}
void Timer0_Delay_Slices(UINT16 s){ (void)s; }
void Timer0_Delay_Ticks(UINT16 t){ (void)t; }
void I2C_Init(void){}

void t0_reset(void) {
    rd = wr = 0; hdr_n = 0; t0_in_n = 0; t0_single_left = 0;
    t0_mode = 0; t0_nulls = 0; t0_outlen = 0; t0_expect_in = 0;
    t0_sw1 = 0x90; t0_sw2 = 0x00; t0_recv_calls = 0;
}
const unsigned char *t0_header(void){ return hdr; }

/* Preload the card to reader queue, and report how much of it was taken, so a
 * test can show a read stopped on the last byte it was owed. */
void t0_push_raw(const unsigned char *b, int n) { int i; for (i = 0; i < n; i++) push(b[i]); }
int  t0_rx_consumed(void) { return rd; }
