#include "i2c.h"

bit BIT_TMP;

void I2C_Init(void) {

    P13_Quasi_Mode;                     /* SCL */
    P14_Quasi_Mode;                     /* SDA */

    SDA = 1;
    SCL = 1;

    /*
     * Leave the port configuration exactly as the working firmware had it.
     *
     * Note for anyone tempted to touch this: P0SR / P1SR are the *slew rate*
     * registers (0xB2 / 0xB4 on page 1).  The Schmitt trigger input select is
     * P0S / P1S (0xB1 / 0xB3).  The comment that used to sit on the line below
     * claimed P0.6 was SCL and that this enabled a Schmitt trigger; both halves
     * were wrong.  P0.6 is UART0 TXD, SCL is P1.3 and SDA is P1.4, and this
     * sets a slew rate.  It is kept because it is what shipped and works, not
     * because it does what it said.
     */
    set_P0SR_6;

    /*
     * I2ADDR bits 7:1 are the slave address and bit 0 enables general call.
     * 0xC0 means address 0x60 with general call off, which is what the
     * Proxmark3 addresses as I2C_DEVICE_ADDRESS_MAIN.
     *
     * Configure first, enable the interrupt last - the old order switched the
     * interrupt on before the address register was written.
     */
    I2ADDR = I2C_DEVICE_ADDRESS_MAIN;

    set_I2CEN;
    set_AA;

    set_EI2C;                           /* enable I2C interrupt, IE1 bit 0 */
    set_EA;
}
