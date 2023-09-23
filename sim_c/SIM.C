/*
 * Kiwi SIM module for Iceman
 */
#include "globals.h"
#include "timer.h"
#include "uart.h"
#include "i2c.h"

UINT8 Buffer[255], cbBuffer = 0, currentIdx = 0, currentCommand = 0;
PCMD_FUNC pCmdFunc = 0;

void main(void)
{
  Set_All_GPIO_Quasi_Mode;
  P10_PushPull_Mode;
  P10 = 1;
  
  CKDIV = 2; // 16 / (2 * CKDIV) = 4 MHz, for smartcard.
  set_CLOEN; // SIM_CLK

  InitialUART0_10800_Parity_Timer1();
  Timer0_Init();
  I2C_Init();
  
  while(1)
  {
    if(pCmdFunc)
    {
      SCL = 0;
      cbBuffer = 0;
      pCmdFunc();
      pCmdFunc = 0;
      SCL = 1;
    }
  }
}

void GETVERSION()
{
  Buffer[0] = 2;
  Buffer[1] = SIM_MODULE_VERS_HI;
  Buffer[2] = SIM_MODULE_VERS_LO;
  cbBuffer = 3;
}

void GENERATE_ATR()
{
  UINT8 i = 0;
  
  P10 = 0;
  Timer0_ResetTime();
  P10 = 1;
  
  clr_RI; REN = 1;
  for(i = 0; Receive_Data_From_UART0_parity_with_timeout(Buffer + 2 + i); i++);
  REN = 0;
  
  Buffer[0] = 0x00;
  Buffer[1] = i;
  cbBuffer = i + 2;
}

void SEND_T0()
{
  UINT8 i, procedureByte, procedureType = 99;
  
  for(i = 0; i < currentIdx; i++)
  {
    if((i < 5) || (procedureType > 2))
    {
      Timer0_UART_Recover();
      Send_Data_To_UART_parity(Buffer[i]);
    }
    
    if((i == 4) || (procedureType == 1) || (procedureType == 4)) // NULL or ACK = INS XOR FF
    {
      clr_RI; REN = 1;
      Receive_Data_From_UART0_parity_with_timeout(&procedureByte);
      REN = 0;
      
      if(procedureByte == 0x60) // problem with i++ here
      {
        procedureType = 1;
      }
      else if(((procedureByte & 0x60) == 0x60) || ((procedureByte & 0x90) == 0x90))
      {
        procedureType = 2;
        Buffer[0] = 0x00;
        Buffer[1] = 2;
        Buffer[2] = procedureByte;
        clr_RI; REN = 1;
        Receive_Data_From_UART0_parity_with_timeout(Buffer + 3);
        REN = 0;
        cbBuffer = 4;
        break;
      }
      else if(procedureByte == Buffer[1])
      {
        procedureType = 3;
      }
      else if(procedureByte == (Buffer[1] ^ 0xff))
      {
        procedureType = 4;
      }
      else
      {
        procedureType = 0;
        Buffer[0] = 0x00;
        Buffer[1] = 2;
        Buffer[2] = 0x42;
        Buffer[3] = 0x42;
        cbBuffer = 4;
        break;
      }
    }
  }
  
  if((procedureType != 0) && (procedureType != 2))
  {
    clr_RI; REN = 1;
    for(i = 0; Receive_Data_From_UART0_parity_with_timeout(Buffer + 2 + i); i++);
    REN = 0;
    
    Buffer[0] = 0x00;
    Buffer[1] = i;
    cbBuffer = i + 2;
  }
}

void I2C_ISR(void) interrupt 6
{
  switch(I2STAT)  
  {
    case 0x00:    // ?
      set_STO;
      break;
    
    /*
     * 15.3.3 - Slave Receiver Mode
     *
     */
    case 0x60:    // Own SLA+W has been received - ACK has been transmitted

      currentCommand = 0;
    
      set_AA;
      break;
 
    case 0x68:    // Arbitration lost and own SLA+W has been received - ACK has been transmitted
      set_AA;
      break;
    
    case 0x80:    // Data byte has been received - ACK has been transmitted
      
      if(!currentCommand)
      {
        currentCommand = I2DAT;
        currentIdx = 0;
      }
      else
      {
        if(currentIdx < sizeof(Buffer))
        {
          Buffer[currentIdx++] = I2DAT;
        }
      }
    
      set_AA;
      break;

    case 0x88:    // Data byte has been received - NACK has been transmitted
      set_AA;
      break;

    /*
     * 15.3.3 - Slave Receiver Mode & 15.3.4 - Slave Transmitter Mode
     *
     */
    case 0xa0:    // A STOP or repeated START has been received
      
      switch(currentCommand)
      {
        case I2C_DEVICE_CMD_GETVERSION:
          GETVERSION();
          break;
        case I2C_DEVICE_CMD_GENERATE_ATR:
          pCmdFunc = GENERATE_ATR;
          break;
        
        case I2C_DEVICE_CMD_SEND:
        case I2C_DEVICE_CMD_SEND_T0:
          pCmdFunc = SEND_T0;
          break;        

        case I2C_DEVICE_CMD_READ:
        default:
          ;
      }
      currentCommand = 0;
    
      set_AA;
      break; 

    /*
     * 15.3.4 - Slave Transmitter Mode
     *
     */
    case 0xa8:    // Own SLA+R has been received - ACK has been transmitted
      
      currentIdx = 0;
      I2DAT = (currentIdx < cbBuffer) ? Buffer[currentIdx++] : 'K';
      // maybe not set_AA if nothing ?
      set_AA;
      break;
    
    case 0xb0:    // Arbitration lost and own SLA+R has been received - ACK has been transmitted
      set_AA;
      break;
    
    case 0xb8:    // Data byte has been transmitted - ACK has been received
      
      I2DAT = (currentIdx < cbBuffer) ? Buffer[currentIdx++] : 'K';
      // maybe not set_AA if nothing ?
      set_AA;
      break; 

    case 0xc0:    // Data byte has been transmitted - NACK has been received
      set_AA;
      break;
    
    case 0xc8:    // Last Data byte has been transmitted - ACK has been received
      set_AA;
      break;
  }
  
  clr_SI;
}