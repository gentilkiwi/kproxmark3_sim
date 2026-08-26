#ifndef __GLOBALS_H__
#define __GLOBALS_H__

#include "N76E003.h"
#include "Common.h"
#include "SFR_Macro.h"
#include "Function_define.h"

#define I2C_DEVICE_ADDRESS_MAIN     0xC0

/* ---- commands the Proxmark3 sends as the first byte after SLA+W ---- */
#define I2C_DEVICE_CMD_GENERATE_ATR 0x01
#define I2C_DEVICE_CMD_SEND         0x02
#define I2C_DEVICE_CMD_READ         0x03
#define I2C_DEVICE_CMD_SETBAUD      0x04
#define I2C_DEVICE_CMD_SIM_CLC      0x05
#define I2C_DEVICE_CMD_GETVERSION   0x06
#define I2C_DEVICE_CMD_SEND_T0      0x07
/* new in v4.51 */
#define I2C_DEVICE_CMD_SEND_T1      0x08  /* plain APDU in, plain APDU out, T=1 */
#define I2C_DEVICE_CMD_PPS          0x09  /* run an ISO 7816-3 clause 9 PPS exchange */

/*
 * v4.50 and older: T=0 only.
 * v4.51        : T=1 block protocol, PPS, working SETBAUD / SIM_CLC,
 *                ATR driven waiting times, I2C slave fixes.
 * v4.52        : reverted the UART divisor and the port slew rate tweak that
 *                4.51 changed; those touched a working signal path and were
 *                never needed for any of the above.
 * v4.53        : PPS reports the Fi/Di actually in force rather than the ATR's
 *                proposal.
 * v4.55        : UART divisor 24 -> 23.  At 24 the stop bit is sampled with
 *                0.16 bits of margin before the next character's start bit,
 *                which is enough for an ATR but not for characters sent back
 *                to back - a T=1 block came back corrupted.
 * v4.54        : the T=1 layer keeps to a time budget.  It could previously
 *                hold the I2C bus for tens of seconds retrying a quiet card,
 *                far past the point the Proxmark3 gives up - which wedges the
 *                bus, since only this module can release SCL.
 */
#define SIM_MODULE_VERS_HI  4
#define SIM_MODULE_VERS_LO  56

/* Every reply to the PM3 is prefixed with a big endian 16 bit length. */
#define PM3_CMD_HEADER_LEN  2
#define TRANSFER_BUF_SIZE   270
#define TRANSFER_MAX_DATA   (TRANSFER_BUF_SIZE - PM3_CMD_HEADER_LEN)

extern UINT8 xdata to_sim[TRANSFER_BUF_SIZE];
extern UINT8 xdata to_pm3[TRANSFER_BUF_SIZE];

#endif // __GLOBALS_H__
