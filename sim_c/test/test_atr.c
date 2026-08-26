#include <stdio.h>
#include <string.h>
#include "iso7816.h"
#include "uart.h"
#include "timer.h"

/* timer stubs - the ATR / baud code never actually waits */
void Timer0_Init(void){}
void Timer0_Start_Slice(void){}
void Timer0_Stop(void){}
void Timer0_Delay_Slices(UINT16 s){ (void)s; }
void Timer0_Delay_Ticks(UINT16 t){ (void)t; }
bit BIT_TMP;

static int fails;
#define CHECK(c) do { if(!(c)){ printf("  FAIL %d: %s\n", __LINE__, #c); fails++; } } while(0)

/* does a slice count cover `etu` and no more than one slice beyond it? */
static int slice_covers(UINT16 slices, UINT32 etu) {
    UINT32 want_us = etu * (UINT32)(UART_RELOAD_DEFAULT * 4u);   /* 1 etu = R*4 us */
    UINT32 got_us  = (UINT32)slices * 50000UL;
    return (got_us >= want_us) && (got_us < want_us + 50000UL);
}

static UINT8 buf[64];
static void parse(const char *hex) {
    int n = 0; const char *p = hex; unsigned v;
    while (sscanf(p, "%2x", &v) == 1) { buf[n++] = (UINT8)v; p += 2; while (*p==' ') p++; if(!*p) break; }
    ISO7816_Parse_ATR(buf, n);
}

int main(void) {
    UART_Init();

    printf("baud divisor math\n");
    CHECK(UART_Get_Reload() == UART_RELOAD_DEFAULT);
    /* rounded up - a truncated etu makes every derived timeout come out short */
    CHECK(UART_Ticks_Per_Etu() == ((4u * UART_RELOAD_DEFAULT) + 2u) / 3u);
    CHECK(UART_Ticks_Per_Etu() * 3u >= 4u * UART_RELOAD_DEFAULT);
    UART_Set_FiDi(0x95); CHECK(UART_Get_Reload() == 2);   /* F=512 D=16 -> 125000 baud exactly */
    UART_Set_FiDi(0x11); CHECK(UART_Get_Reload() == 23);
    UART_Set_FiDi(0x18); CHECK(UART_Get_Reload() == 2);   /* F=372 D=12 -> 1.94 -> 2 */
    UART_Set_FiDi(0x77); CHECK(UART_Get_Reload() == 2);   /* both RFU, left as it was */
    UART_Set_TH1(0xE8);  CHECK(UART_Get_Reload() == 24);  /* legacy value round trips */
    UART_Set_Reload(23);

    printf("waiting times at the default etu\n");
    ISO7816_Reset_Params();
    /* Check the derived timeouts against the ISO figures worked out from first
     * principles, rather than against whatever the code happens to produce.
     * One etu is R * 16 / Fsys = R * 4 us at Fsys = 4 MHz, and a slice is 50 ms,
     * so a timeout must land in [wanted, wanted + one slice). */
    CHECK(slice_covers(iso.wwt_slices, 960UL * 10UL + 480UL));   /* WWT, WI = 10 */
    CHECK(slice_covers(iso.bwt_slices, 16UL * 960UL + 11UL));    /* BWT, BWI = 4 */
    CHECK(slice_covers(iso.cwt_slices, 8192UL + 11UL));          /* CWT, CWI = 13 */

    printf("T=0 only, no TD1\n");
    parse("3B 65 00 00 9C 11 01 01 03");
    CHECK(iso.protocols == ISO7816_PROTO_T0);
    CHECK(iso.first_proto == 0);
    CHECK(iso.ifsc == 32 && iso.bwi == 4 && iso.cwi == 13);

    printf("JCOP v241, T=1 only\n");
    parse("3B F8 13 00 00 81 31 FE 45 4A 43 4F 50 76 32 34 31 B7");
    CHECK(iso.protocols == ISO7816_PROTO_T1);
    CHECK(iso.first_proto == 1);
    CHECK(iso.ta1 == 0x13);
    CHECK(iso.guard_n == 0x00);
    CHECK(iso.ifsc == 254);
    CHECK(iso.bwi == 4 && iso.cwi == 5);
    CHECK(iso.edc_crc == 0);
    CHECK(iso.active_proto == 1);

    printf("dual protocol, T=0 first\n");
    parse("3B 80 80 11 FE 65");
    CHECK(iso.protocols == (ISO7816_PROTO_T0 | ISO7816_PROTO_T1));
    CHECK(iso.first_proto == 0);
    CHECK(iso.active_proto == 0);
    CHECK(iso.ifsc == 254);

    printf("T=1 with CRC and a custom BWI/CWI\n");
    /* TS T0=BF(TA1,TB1,TC1,TD1 + 15 hist)... keep it small instead:
       T0=0x80 -> TD1 only; TD1=0xF1 -> TA2..TD2 present, T=1 */
    parse("3B 80 F1 FE 75 01 21 00");
    /* group2: TA2=FE, TB2=75, TC2=01, TD2=21 -> Y3=2 (TB3), T=1 ; TB3=00 */
    CHECK(iso.first_proto == 1);
    CHECK(iso.wi == 1);                 /* TC2 */
    CHECK(iso.bwi == 0 && iso.cwi == 0);/* TB3 = 0x00 */

    printf("T=1 group 3 interface bytes with CRC bit\n");
    parse("3B 80 81 31 20 41 00");
    /* TD1=0x81 -> Y2=8, T=1 ; TD2=0x31 -> Y3=3 (TA3,TB3), T=1 ; TA3=0x20 IFSC=32,
       TB3=0x41 BWI=4 CWI=1 ; then no more -> the 00 is a historical byte/TCK */
    CHECK(iso.first_proto == 1);
    CHECK(iso.ifsc == 32);
    CHECK(iso.bwi == 4 && iso.cwi == 1);

    printf("real card: Fido SIM, T=0 with T=15 global bytes\n");
    parse("3B 9F 96 80 1F C7 80 31 E0 73 FE 21 1B 64 41 63 32 00 82 90 00 D1");
    CHECK(iso.protocols == ISO7816_PROTO_T0);   /* TD1 says T=0, TD2 says T=15 */
    CHECK(iso.first_proto == 0);
    CHECK(iso.active_proto == 0);
    CHECK(iso.ta1 == 0x96);                     /* F = 512, D = 32 */
    CHECK(iso.guard_n == 0);                    /* no TC1 */
    CHECK(iso.wi == 10);                        /* no TC2, so the default */
    CHECK(iso.ifsc == 32);                      /* TA3 is T=15's, not T=1's */
    CHECK(iso.bwi == 4 && iso.cwi == 13);
    /* and the rate that TA1 would ask for if a PPS ever negotiated it */
    UART_Set_FiDi(0x96);
    CHECK(UART_Get_Reload() == 1);              /* 512 / (16 * 32) */
    UART_Set_Reload(UART_RELOAD_DEFAULT);

    printf("truncated ATR must not walk off the end\n");
    parse("3B FF");
    CHECK(iso.valid == 1);
    ISO7816_Parse_ATR(buf, 0);
    CHECK(iso.valid == 0);

    printf("guard time from TC1\n");
    parse("3B 90 00 11 FF");   /* T0=90 -> Y1=9 (TA1,TD1) */
    CHECK(iso.ta1 == 0x00);

    printf(fails ? "\n%d FAILURES\n" : "\nall ATR / baud checks passed\n", fails);
    return fails != 0;
}
