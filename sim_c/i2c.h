#ifndef __I2C_H__
#define __I2C_H__
#include "globals.h"

// Direct Call
#define I2C_BUS_ERR                 0x00
#define I2C_RECEIVE_ACK             0x60  // Own SLA+W has been received - ACK has been transmitted
#define I2C_RECEIVE_ABR_ERR         0x68  // Arbitration lost and own SLA+W has been received - ACK has been transmitted
#define I2C_RECEIVE_DATA_OK         0x80  // Data byte has been received - ACK has been transmitted
#define I2C_RECEIVE_DATA_ERR        0x88  // Data byte has been received - NACK has been transmitted
#define I2C_STOP_START              0xA0  // A STOP or repeated START has been received
#define I2C_TRANSMIT_ADR_OK         0xA8  // Own SLA+R has been received - ACK has been transmitted
#define I2C_TRANSMIT_ABR_ERR        0xB0  // Arbitration lost and own SLA+R has been received - ACK has been transmitted
#define I2C_TRANSMIT_DATA_OK        0xB8  // Data byte has been transmitted - ACK has been received
#define I2C_TRANSMIT_DATA_ERR       0xC0  // Data byte has been transmitted - NACK has been received
#define I2C_TRANSMIT_STOP           0xC8  // Last Data byte has been transmitted - ACK has been received

// General Call
#define I2C_GC_RECEIVE_ACK          0x70  // General Call - Own SLA+W has been received - ACK has been transmitted
#define I2C_GC_RECEIVE_ABR_ERR      0x78  // General Call - Arbitration lost and own SLA+W has been received - ACK has been transmitted
#define I2C_GC_RECEIVE_DATA_OK      0x90  // General Call - Data byte has been received - ACK has been transmitted
#define I2C_GC_RECEIVE_DATA_ERR     0x98  // General Call - Data byte has been received - NACK has been transmitted

void I2C_Init();

#endif // __I2C_H__