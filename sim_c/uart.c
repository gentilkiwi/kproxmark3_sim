#include "uart.h"
#include "timer.h"

void InitialUART0_10800_Parity_Timer1()//UINT32 u32Baudrate)    //T1M = 1, SMOD = 1
{
  SCON = 0x50;     	//UART0 Mode1,REN=1,TI=1
  TMOD |= 0x20;    	//Timer1 Mode1
  set_SMOD;   //UART0 Double Rate Enable
  set_T1M;
  clr_BRCK;   //Serial port 0 baud rate clock source = Timer1
  TH1 = 0xe8; //256 - ((1000000/4)/u32Baudrate+1);               /*16 MHz -> 4 Mhz*/
  set_TR1;
  set_FE;
  REN = 0;
}

void Send_Data_To_UART_parity(UINT8 c)
{
  ACC = c;
  TB8 = P;
  clr_TI;
  SBUF = c;
  while(!TI);
}

UINT8 Receive_Data_From_UART0_parity()
{
  UINT8 c;

  while (!RI);
  c = SBUF;
  // TODO, check parity bit TB8 == P
  clr_RI;
  return c;
}

UINT8 Receive_Data_From_UART0_parity_with_timeout(UINT8 * pChar)
{
  UINT8 ret = 0;

  clr_RI;
  REN = 1;
    
  Timer0_Start_Timeout();
  while (!RI && !TF0);
  Timer0_Stop_Timeout();

  if(RI)
  {
    *pChar = SBUF;
    //= ACC;
    // TODO, check parity bit TB8 == P
    clr_RI;
    ret = 1;
  }
  
  REN = 0;
  
  return ret;
}
