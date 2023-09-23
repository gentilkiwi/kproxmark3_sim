#include "i2c.h"

bit BIT_TMP;

void I2C_Init()
{
  P13_Quasi_Mode;
  P14_Quasi_Mode;

  SDA = 1;
  SCL = 1;
  
  set_P0SR_6; //set SCL (P06) is  Schmitt triggered input select.
  set_EI2C;   //enable I2C interrupt by setting IE1 bit 0
  set_EA;

  I2ADDR = I2C_DEVICE_ADDRESS_MAIN;
  set_I2CEN;
  set_AA;
}