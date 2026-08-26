#ifndef __ISO7816_H__
#define __ISO7816_H__
#include "globals.h"

#define ISO7816_PROTO_T0    0x01
#define ISO7816_PROTO_T1    0x02

/*
 * Everything the ATR told us about how to talk to this card.  The module used
 * to hard code a 100 ms timeout for every wait, which is both too long for an
 * inter character gap and far too short for the first byte of an answer:
 * a default T=0 card may take WWT ~= 0.9 s and a default T=1 card BWT ~= 1.4 s
 * before it says anything.  These are now derived from the ATR.
 */
typedef struct {
    UINT8  valid;         /* an ATR has been parsed                          */
    UINT8  protocols;     /* bit n set = T=n offered                         */
    UINT8  first_proto;   /* protocol named by TD1, the one in force at reset*/
    UINT8  active_proto;  /* what we are actually speaking now               */
    UINT8  ta1;           /* FI << 4 | DI                                    */
    UINT8  guard_n;       /* TC1, extra guard time                           */
    UINT8  wi;            /* TC2, T=0 waiting time integer, default 10       */
    UINT8  ifsc;          /* T=1 TA(i>2), card's max INF size, default 32    */
    UINT8  bwi;           /* T=1 TB(i>2) high nibble, default 4              */
    UINT8  cwi;           /* T=1 TB(i>2) low nibble, default 13              */
    UINT8  edc_crc;       /* T=1 TC(i>2) bit 0: 1 = CRC, 0 = LRC             */
    UINT16 wwt_slices;    /* T=0 work waiting time, in 50 ms slices          */
    UINT16 cwt_slices;    /* T=1 character waiting time                      */
    UINT16 bwt_slices;    /* T=1 block waiting time                          */
} iso7816_t;

extern iso7816_t xdata iso;

void ISO7816_Reset_Params(void);
void ISO7816_Update_Timeouts(void);
void ISO7816_Parse_ATR(UINT8 xdata *atr, UINT16 len);

/* ISO/IEC 7816-3 clause 9.  Returns 1 when the card confirmed the request.
 * When use_ta1 is 0 only the protocol is negotiated and the timing stays as
 * the ATR left it. */
UINT8 ISO7816_PPS(UINT8 proto, UINT8 ta1, UINT8 use_ta1);

#endif // __ISO7816_H__
