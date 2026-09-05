/* Host checks for the interrupt-fed UART receive queue. */
#include <stdio.h>
#include "uart.h"
#include "timer.h"

void UART_ISR(void);
bit BIT_TMP;
static int fails;
#define CHECK(c) do { if (!(c)) { printf("FAIL %d: %s\n", __LINE__, #c); fails++; } } while (0)

void Timer0_Init(void) {}
void Timer0_Start_Slice(void) { TF0 = 1; }
void Timer0_Stop(void) { TF0 = 0; }
void Timer0_Delay_Slices(UINT16 s) { (void)s; }
void Timer0_Delay_Ticks(UINT32 t) { (void)t; }

static void receive(UINT8 c, UINT8 bad_parity) {
    UINT8 v = c;
    UINT8 parity = 0;
    while (v) { parity ^= (v & 1u); v >>= 1; }
    SBUF = c;
    /* Host SFR stand-ins do not derive PSW.P from ACC. */
    P = parity;
    RB8 = parity ^ bad_parity;
    RI = 1;
    UART_ISR();
    CHECK(RI == 0);
}

int main(void) {
    UINT8 buf[64];
    UINT8 c;
    unsigned i, round;
    UART_Init();
    UART_Set_FiDi(0x96);
    CHECK(UART_Get_Reload() == 4);

    /* Procedure byte and data can arrive before the caller changes paths. */
    receive(0xB0, 0);
    receive(0x12, 0);
    receive(0x90, 0);
    receive(0x00, 0);
    CHECK(UART_Recv(&c, 1) == 1 && c == 0xB0);
    CHECK(UART_Recv_Burst(buf, 3, 1, 1) == 3);
    CHECK(buf[0] == 0x12 && buf[1] == 0x90 && buf[2] == 0);

    /* Repeated blocks wrap both indices without losing order. */
    for (round = 0; round < 20; round++) {
        for (i = 0; i < 37; i++) receive((UINT8)(round + i), 0);
        CHECK(UART_Recv_Burst(buf, 37, 1, 1) == 37);
        for (i = 0; i < 37; i++) CHECK(buf[i] == (UINT8)(round + i));
    }
    CHECK(!uart_parity_err);
    CHECK(UART_Recv_Burst(buf, 1, 1, 1) == 0);
    receive(0x55, 1);
    CHECK(uart_parity_err);
    CHECK(UART_Recv(&c, 1) && c == 0x55);

    uart_parity_err = 0;
    for (i = 0; i < 64; i++) receive((UINT8)i, 0);
    CHECK(uart_parity_err);
    CHECK(UART_Recv_Burst(buf, 64, 1, 1) == 63);
    for (i = 0; i < 63; i++) CHECK(buf[i] == i);
    receive(0xAA, 0);
    UART_Rx_Reset();
    CHECK(ES == 0 && REN == 0);
    CHECK(UART_Recv(&c, 1) == 0);
    printf("UART queue: %d failures\n", fails);
    return fails != 0;
}
