/*
 * Kiwi SIM module for Iceman
 */
#include "globals.h"
#include "timer.h"
#include "uart.h"
#include "i2c.h"

// global buffer
UINT8 send_buf[260];
UINT8 send_size = 0;
UINT16 curr_idx = 0;

// shouldn't it be a ENUM?
UINT8 curr_cmd = 0;

#define CMD_POINTER_ZERO  0
PCMD_FUNC pCmdFunc = CMD_POINTER_ZERO;


/*

ICEMAN;
What is the structure for send_buf ?
May I suggest we impose a struct on top of it instead? 
*/

void main(void) {
    
    // zero out send buffer
    int i;
    
    for (i = 0; i < sizeof(send_buf); i += 4) {
        send_buf[i] = 0x00;
        send_buf[i + 1] = 0x00;
        send_buf[i + 2] = 0x00;
        send_buf[i + 3] = 0x00;
    }

    Set_All_GPIO_Quasi_Mode;
    P10_PushPull_Mode;
    P10 = 1;

    CKDIV = 2; // 16 / (2 * CKDIV) = 4 MHz, for smartcard.
    set_CLOEN; // SIM_CLK

    InitialUART0_10800_Parity_Timer1();
    Timer0_Init();
    I2C_Init();

    // main loop
    while (1) {
        // if pointer to function is set,  
        if (pCmdFunc) {
            // SET CLOCK LINE - LOW
            SCL = 0;

            // reset send_size
            send_size = 0;

            pCmdFunc();

            // reset pointer to function
            pCmdFunc = CMD_POINTER_ZERO;

            // SET CLOCK LINE - HIGH
            SCL = 1;
        }
    }
}

// SEND ACK?
static void SEND_ACK(UINT8 i) {
    send_buf[0] = 0x00;  // ?
    send_buf[1] = i;
    send_size = i + 2;
}

// Send Firmware Version to Proxmark3
void GETVERSION() {
    send_buf[0] = 2;  // I2C_DEVICE_CMD_SEND  ??
    send_buf[1] = SIM_MODULE_VERS_HI;
    send_buf[2] = SIM_MODULE_VERS_LO;
    send_size = 3;
}



void GENERATE_ATR() {

    UINT8 i = 0;  

    // set PIN 10 = LOW ?  
    P10 = 0;

    Timer0_ResetTime();

    // set PIN 10 = HIGH ?  
    P10 = 1;

    // OFFSET = 2 + i
    // i = Num of bytes
    for(i = 0; Receive_Data_From_UART0_parity_with_timeout(send_buf + 2 + i); i++);
  
	SEND_ACK(i);
}

void SEND() {
    UINT8 i;

    for(i = 0; i < curr_idx; i++) {
        Send_Data_To_UART_parity(send_buf[i]);
    }

    for(i = 0; Receive_Data_From_UART0_parity_with_timeout(send_buf + 2 + i); i++);

	SEND_ACK(i);
}

// send debug message to Proxmark3
static void send_debug(UINT8 i, UINT8 ins, UINT8 procedurebyte) {
    send_buf[0] = 0x00;  // ?
    send_buf[1] = 0x05;  // length of package
    send_buf[2] = 0x42;
    send_buf[3] = 0x42;
    send_buf[4] = i;
    send_buf[5] = ins;
    send_buf[6] = procedurebyte;
    send_size = 7;
}

/*
ICEMAN
I find it always unclear which these names.
are we sending to pm3?  or to SIM card?
*/
void SEND_T0() {

    UINT8 i, j = 0;
    UINT8 procedureByte = 0;
    UINT8 ins = send_buf[1];
    UINT8 tmp;

    // Send to SIM - apdu 5 bytes header
    for (i = 0; i < 5; i++) {
        Send_Data_To_UART_parity(send_buf[j]);
        j++;
    }

    // Five bytes send to SIM, now waiting for procedure byte.
    
    for (; i < curr_idx; i++) {
       
        // recieve from SIM
        if (Receive_Data_From_UART0_parity_with_timeout(&procedureByte) == 0) {
            SEND_ACK(j);
            return;
        }
        
        // ACK 10.3.3
        if (procedureByte == (ins ^ 0xFF)) {

            Timer0_UART_Recover();            

            // send one byte
            Send_Data_To_UART_parity(send_buf[j]);
            j++;
            continue;
        }
        
        // NULL recieved,  wait until new protocol byte comes from SIM.
        while (procedureByte == 0x60) {
            // recieve from SIM
            if (Receive_Data_From_UART0_parity_with_timeout(&procedureByte) == 0) {
                return;
            }
        }


        // SW0? check for 0x6[1-F] and 0x9[0-F] 
        // When here,  procedure byte can never be the exception 0x60.  Already dealt w above.
        tmp = ((procedureByte >> 4) & 0x0F);
        if ((tmp == 0x6) || (tmp == 0x9)) {

            send_buf[0] = 0x00;  // ?
            send_buf[1] = 0x02;  // SW0 + SW1
            send_buf[2] = procedureByte;

            // receive SW1
            Receive_Data_From_UART0_parity_with_timeout(send_buf + 3);
                
            send_size = 4;

            // After valid SW,  quite loop
            return;
        }


        // ACK 10.3.3 
        if (procedureByte == ins) {

            // Send to SIM rest of the data
            for(; i < curr_idx; i++) {
                Send_Data_To_UART_parity(send_buf[j]);
                j++;
            }
            
            // read the rest of the data...
            for(; Receive_Data_From_UART0_parity_with_timeout(send_buf + 2 + j); i++, j++);
    
            SEND_ACK(j);
            return;
        }

        send_debug(j, 0xBB, procedureByte);
       
    } // end for   
}


/*
ICEMAN,
I tried to understand the different I2C we are looking for

#define I2STAT_ZERO           0x00
#define I2STAT_ACK            0x60 // Own SLA+W has been received - ACK has been transmitted		
#define I2STAT_LOST_ABR_ACK   0x68 // Arbitration lost and own SLA+W has been received - ACK has been transmitted
#define I2STAT_DATA_RECIEVED  0x80 // Data byte has been received - ACK has been transmitted
#define I2STAT_DATA_NACK      0x88 // Data byte has been received - NACK has been transmitted
#define I2STAT_DATA_STOP_     0xA0 // A STOP or repeated START has been received
#define I2STAT_               0xA8 // Own SLA+R has been received - ACK has been transmitted
#define I2STAT_               0xB0 // Arbitration lost and own SLA+R has been received - ACK has been transmitted
#define I2STAT_               0xB8 // Data byte has been transmitted - ACK has been received
#define I2STAT_               0xC0 // Data byte has been transmitted - NACK has been received
#define I2STAT_LAST_DATA      0xC8 // Last Data byte has been transmitted - ACK has been received

*/

// Interrupt Handler IRQ 6 
void I2C_ISR(void) interrupt 6
{
    switch(I2STAT) {
        case 0x00: {   // ?
            set_STO;
            break;
        }
    
        /*
         * 15.3.3 - Slave Receiver Mode
         *
         */
        // Own SLA+W has been received - ACK has been transmitted		
        case 0x60: {
            curr_cmd = CMD_POINTER_ZERO;
            set_AA;
            break;
        }
		
		// Arbitration lost and own SLA+W has been received - ACK has been transmitted
        case 0x68: {
            set_AA;
            break;
        }
    
        // Data byte has been received - ACK has been transmitted
        case 0x80: {
      
            if (curr_cmd == CMD_POINTER_ZERO) {
                curr_cmd = I2DAT;
                curr_idx = 0;
            } else {
               
                // receive data from PROXMARK3.
                if (curr_idx < sizeof(send_buf))	{
                    send_buf[curr_idx++] = I2DAT;
                }
            }
            set_AA;
            break;
        }

        // Data byte has been received - NACK has been transmitted
        case 0x88: {
          set_AA;
          break;
		}

/*
 * 15.3.3 - Slave Receiver Mode & 15.3.4 - Slave Transmitter Mode
 *
 */
		// A STOP or repeated START has been received
        case 0xA0: {

			switch(curr_cmd) {
                case I2C_DEVICE_CMD_GETVERSION: {
                    GETVERSION();
                    break;
                }
                case I2C_DEVICE_CMD_GENERATE_ATR: {
                    pCmdFunc = GENERATE_ATR;
                    break;
                }
                case I2C_DEVICE_CMD_SEND: {
                    pCmdFunc = SEND;
                    break;
                }
                case I2C_DEVICE_CMD_SEND_T0: {
                     pCmdFunc = SEND_T0;
                     break;        
                }
                case I2C_DEVICE_CMD_READ:
                default: { 
                     break;
                }
            }

            curr_cmd = CMD_POINTER_ZERO;
            set_AA;
            break; 
		}
/*
 * 15.3.4 - Slave Transmitter Mode
 *
 */
        // Own SLA+R has been received - ACK has been transmitted
        case 0xa8: {
      
            curr_idx = 0;
            I2DAT = (curr_idx < send_size) ? send_buf[curr_idx++] : 'K';
            // maybe not set_AA if nothing ?
            set_AA;
            break;
        }
        // Arbitration lost and own SLA+R has been received - ACK has been transmitted
        case 0xb0: {
            set_AA;
            break;
        }
        // Data byte has been transmitted - ACK has been received
        case 0xb8: {
  
            I2DAT = (curr_idx < send_size) ? send_buf[curr_idx++] : 'K';
            // maybe not set_AA if nothing ?
            set_AA;
            break; 
		}
        // Data byte has been transmitted - NACK has been received
        case 0xc0: {
            set_AA;
            break;
		}
        // Last Data byte has been transmitted - ACK has been received
        case 0xc8: {
            set_AA;
            break;
		}
    }

    // clear ?
    clr_SI;
}