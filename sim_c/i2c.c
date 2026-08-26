#include "i2c.h"

bit BIT_TMP;

void I2C_Init(void) {

    P13_Quasi_Mode;                     /* SCL */
    P14_Quasi_Mode;                     /* SDA */

    SDA = 1;
    SCL = 1;

    /*
     * P0SR/P1SR are the slew rate registers; Schmitt trigger select is P0S/P1S.
     * P0.6 is UART0 TXD, not SCL (P1.3). Kept as-is because it is what shipped
     * and works, not because it does what its old comment claimed.
     */
    set_P0SR_6;

    // I2ADDR bits 7:1 are the address, bit 0 the general call enable.
    // Configure first, enable the interrupt last.
    I2ADDR = I2C_DEVICE_ADDRESS_MAIN;

    set_I2CEN;
    set_AA;

    set_EI2C;                           /* enable I2C interrupt, IE1 bit 0 */
    set_EA;
}
