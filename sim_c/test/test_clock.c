/* Clock and baud-register checks against the firmware, without card hardware. */
#include <stdio.h>
#include "uart.h"
#include "timer.h"
#include "iso7816.h"

bit BIT_TMP;
static int fails;
#define CHECK(c) do { if (!(c)) { printf("FAIL %d: %s\n", __LINE__, #c); fails++; } } while (0)

static UINT32 timer_divisor(void) {
    UINT32 count = 65536UL - ((UINT32)RH3 * 256UL + RL3);
    return count << (T3CON & 7u);
}

int main(void) {
    UINT8 i;
    static const UINT8 ta1[] = {0x11, 0x92, 0x93, 0x94, 0x95, 0x96};
    static const UINT16 counts[] = {93, 64, 32, 16, 8, 4};
    UINT16 default_wait;

    UART_Clock_Init();
    UART_Init();
    Timer0_Init();
    CHECK(CKDIV == 0);
    CHECK((CKCON & 0x42) == 0);           /* no CLO, PWM uses Fsys */
    CHECK(PWMPH == 0 && PWMPL == 3);
    CHECK(PWM1H == 0 && PWM1L == 2);
    CHECK(PIOCON0 == 2);                 /* P1.1 only */
    CHECK((PIOCON1 & 2) == 0);           /* leave I2C SDA alone */
    CHECK(PWMCON1 == 0);
    CHECK((T3CON & 0x28) == 0x28);

    for (i = 0; i < sizeof(ta1); i++) {
        CHECK(UART_FiDi_Supported(ta1[i]));
        UART_Set_FiDi(ta1[i]);
        CHECK(timer_divisor() == counts[i]);
        CHECK(UART_Get_Reload() == counts[i]);
    }
    CHECK(SIM_FSYS_HZ / (16UL * timer_divisor()) == 250000UL);
    CHECK(!UART_FiDi_Supported(0x77));
    UART_Set_FiDi(0x77);
    CHECK(timer_divisor() == 4);

    /* ARM firmware restores TA1=96 by writing legacy TH1=FF. */
    UART_Set_TH1(0xFF);
    CHECK(timer_divisor() == 4);
    UART_Set_TH1(0xFE);
    CHECK(timer_divisor() == 8);
    UART_Set_TH1(0x00);
    CHECK(timer_divisor() == 1024);

    UART_Set_FiDi(0x96);
    CHECK(!UART_Set_Clock(0));            /* would require forbidden divisor 1 */
    CHECK(PWMPL == 3 && timer_divisor() == 4);
    CHECK(UART_Set_Clock(4));             /* legacy CKDIV=4: 2 MHz card */
    CHECK(PWMPL == 7 && PWM1L == 4);
    CHECK(timer_divisor() == 8);
    CHECK(UART_Set_Clock(2));
    ISO7816_Reset_Params();
    CHECK(timer_divisor() == 93);
    default_wait = iso.wwt_slices;
    UART_Set_FiDi(0x96);
    ISO7816_Update_Timeouts();
    CHECK(iso.wwt_slices >= default_wait);
    CHECK(UART_Etu_To_Slices(0xFFFFFFFFUL) == T0_MAX_SLICES);

    /* Long legacy divisors must fit without silent 8/16-bit truncation. */
    CHECK(UART_Set_Clock(255));
    UART_Set_TH1(0);
    CHECK(timer_divisor() == 130560UL);
    CHECK(UART_Ticks_Per_Etu() == 174080UL);
    CHECK(UART_Etu_To_Slices(10080UL) == T0_MAX_SLICES);
    CHECK(UART_Set_Clock(2));
    ISO7816_Reset_Params();

    Timer0_Start_Slice();
    CHECK(65536UL - ((UINT32)TH0 * 256UL + TL0) == 33334UL);
    CHECK(T0_SLICE_TICKS * 12000UL >= SIM_FSYS_HZ * T0_SLICE_MS);
    CHECK(T0_SLICE_TICKS * 12000UL - SIM_FSYS_HZ * T0_SLICE_MS < 12000UL);
    Timer0_Stop();
    printf("Clock and baud: %d failures\n", fails);
    return fails != 0;
}
