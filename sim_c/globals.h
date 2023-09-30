#ifndef __GLOBALS_H__
#define __GLOBALS_H__

#include "N76E003.h"
#include "Common.h"
#include "SFR_Macro.h"
#include "Function_define.h"

#define I2C_DEVICE_ADDRESS_MAIN     0xC0

#define I2C_DEVICE_CMD_GENERATE_ATR 0x01
#define I2C_DEVICE_CMD_SEND         0x02
#define I2C_DEVICE_CMD_READ         0x03
#define I2C_DEVICE_CMD_SETBAUD      0x04
#define I2C_DEVICE_CMD_SIM_CLC      0x05
#define I2C_DEVICE_CMD_GETVERSION   0x06
#define I2C_DEVICE_CMD_SEND_T0      0x07

#define SIM_MODULE_VERS_HI  4
#define SIM_MODULE_VERS_LO  50

typedef void(*PCMD_FUNC) ();

#endif // __GLOBALS_H__