#ifndef __UART_H__
#define __UART_H__
#include "globals.h"

void InitialUART0_10800_Parity_Timer1();

void Send_Data_To_UART_parity(UINT8 c);
UINT8 Receive_Data_From_UART0_parity();
UINT8 Receive_Data_From_UART0_parity_with_timeout(UINT8 * pChar);

#endif // __UART_H__