#ifndef __T1_H__
#define __T1_H__
#include "globals.h"

/*
 * ISO 7816-3 clause 11, T=1 block protocol:  NAD PCB LEN [INF 0..254] EDC,
 * EDC being an LRC or a CRC when TCi says so. The module owns the block layer,
 * so the PM3 hands over a plain APDU and gets one back, as with SEND_T0.
 *
 * Covers chaining both ways, R block retransmission, S(IFS), S(WTX), S(ABORT)
 * and S(RESYNCH). Nothing allocates - blocks stream through to_sim/to_pm3 with
 * the checksum computed on the fly, since XRAM is 768 bytes and the two
 * transfer buffers take 540.
 */

/*
 * The PM3 gives up on a stretched SCL after ~3 s and only this module can
 * release it, so overrunning wedges the bus rather than just losing the
 * exchange. Every wait comes out of one budget and BWT is clamped to what is
 * left: a card slower than the host is patient cannot be served either way.
 */
#define T1_BUDGET_SLICES        56      /* 2.8 s in 50 ms slices */

// S(IFS) is answered from card state, so it arrives at once or not at all.
// One attempt; a card without it just keeps the default IFSD.
#define T1_HOUSEKEEPING_SLICES  8       /* 400 ms */

/* Likewise a PPS response - the card is not computing anything. */
#define T1_PPS_SLICES           20      /* 1 s */

#define T1_E_TIMEOUT    (-1)
#define T1_E_PROTO      (-2)
#define T1_E_OVERFLOW   (-3)
#define T1_E_ABORT      (-4)
#define T1_E_RESYNCH    (-5)
#define T1_E_PARAM      (-6)

/* What we tell the card we can swallow in one block.  254 is the maximum and
 * keeps most answers out of receive chaining altogether; to_pm3 has room for
 * it.  Set to 32 to skip the S(IFS) exchange entirely. */
#define T1_IFSD_WANTED  254

void  T1_Reset(void);
void  T1_Begin(void);
void  T1_Prepare(void);
UINT8 T1_Negotiate_IFSD(UINT8 ifsd);

INT16 T1_Transceive(UINT8 xdata *tx, UINT16 txlen, UINT8 xdata *rx, UINT16 rxmax);

#endif // __T1_H__
