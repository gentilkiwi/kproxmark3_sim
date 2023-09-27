/*
 * Kiwi SIM module for Iceman
 */
#include "globals.h"
#include "timer.h"
#include "uart.h"
#include "i2c.h"


#define TRANSFER_BUF_SIZE   260

// Buffer for data to be send to SIM CARD
UINT8 to_sim[TRANSFER_BUF_SIZE];

// Buffer for data to be send to PM3
UINT8 to_pm3[TRANSFER_BUF_SIZE];


// how long APDU message to send to SIM CARD
UINT16 curr_sim_len = 0;
// how long message to send to PM3
UINT16 to_pm3_len = 0;   

// Which byte are we currently sending to PM3
UINT16 curr_send_idx = 0;

// shouldn't it be a ENUM?
UINT8 curr_cmd = 0;

#define CMD_POINTER_ZERO  0
PCMD_FUNC pCmdFunc = CMD_POINTER_ZERO;

#define PM3_CMD_HEADER_LEN    2

/*

ICEMAN;
What is the structure for send_buf ?
May I suggest we impose a struct on top of it instead? 
*/

void main(void) {
    
    // zero buffers
    int i;
    
    for (i = 0; i < TRANSFER_BUF_SIZE; i += 4) {
        to_sim[i] = 0x00;
        to_sim[i + 1] = 0x00;
        to_sim[i + 2] = 0x00;
        to_sim[i + 3] = 0x00;

        to_pm3[i] = 0x00;
        to_pm3[i + 1] = 0x00;
        to_pm3[i + 2] = 0x00;
        to_pm3[i + 3] = 0x00;
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
           
            pCmdFunc();

            // reset pointer
            pCmdFunc = CMD_POINTER_ZERO;

            // SET CLOCK LINE - HIGH
            SCL = 1;
        }
    }
}

static void queue_pm3(UINT16 i) {
    to_pm3[0] = ((i >> 8) & 0xFF); // MSB length of package
    to_pm3[1] = (i & 0xFF);        // LSB length of package
    to_pm3_len = PM3_CMD_HEADER_LEN + i;
}


static void queue_sw_to_pm3(UINT8 sw0) {
    
    to_pm3[0] = 0;    // MSB length of package
    to_pm3[1] = 2;    // LSB length of package
    to_pm3[2] = sw0;

    // receive SW1
    Receive_Data_From_UART0_parity_with_timeout(&to_pm3[3]);

    to_pm3_len = PM3_CMD_HEADER_LEN + 2;
}


// Send Firmware Version to Proxmark3
static void GETVERSION() {
    to_pm3[0] = 0;    // MSB length of package
    to_pm3[1] = 2;    // LSB length of package
    to_pm3[2] = SIM_MODULE_VERS_HI;
    to_pm3[3] = SIM_MODULE_VERS_LO;
    to_pm3_len = PM3_CMD_HEADER_LEN + 2;
}


static void GENERATE_ATR() {

    UINT16 i = 0;  

    // set PIN 10 = LOW ?  
    P10 = 0;

    Timer0_ResetTime();

    // set PIN 10 = HIGH ?  
    P10 = 1;

//    for(; Receive_Data_From_UART0_parity_with_timeout(&to_pm3[PM3_CMD_HEADER_LEN + i]) && (i < sizeof(to_pm3)); i++);
    for(; Receive_Data_From_UART0_parity_with_timeout(&to_pm3[PM3_CMD_HEADER_LEN + i]); i++);
	queue_pm3(i);
}


// SEND to SIM card
// receive for SIM card and SEND to PM3
void SEND() {
    UINT16 i;

    for(i = 0; i < curr_sim_len; i++) {
        Send_Data_To_UART_parity(to_sim[i]);
    }

    // for(i = 0; Receive_Data_From_UART0_parity_with_timeout(to_pm3 + PM3_CMD_HEADER_LEN + i) && (i < sizeof(to_pm3)); i++);
    for(i = 0; Receive_Data_From_UART0_parity_with_timeout(&to_pm3[PM3_CMD_HEADER_LEN + i]); i++);
	queue_pm3(i);
}


void SEND_T0() {

    UINT8 si;
    UINT8 pi = 0;
    UINT8 procedureByte = 0;
    UINT8 ins = to_sim[1];
    UINT8 tmp;
    
    // Sanity CHECK
    if (curr_sim_len < 5) {
        return;
    }

    // Send to SIM - APDU 5 bytes header
    for (si = 0; si < 5; si++) {
        Send_Data_To_UART_parity(to_sim[si]);
    }

    // Five bytes send to SIM, now waiting for procedure byte.
    
    for (; si < curr_sim_len; si++) {
       
        loop:
        
        if (Receive_Data_From_UART0_parity_with_timeout(&procedureByte) == 0) {
            // if failure to recieve from SIM, send what we got back to PM3
            queue_pm3(pi);
            return;
        }

        // NULL recieved,  wait until new protocol byte comes from SIM.
        if (procedureByte == 0x60) {
            goto loop;
        }
        
        Timer0_UART_Recover();
        
        // ACK 10.3.3
        if (procedureByte == (ins ^ 0xFF)) {

            // send one byte
            Send_Data_To_UART_parity(to_sim[si]);
            continue;
        }

        
        // SW0? check for 0x6[1-F] and 0x9[0-F] 
        // When here,  procedure byte can never be the exception 0x60.  Already dealt w above.
        tmp = ((procedureByte >> 4) & 0x0F);
        if ((tmp == 0x6) || (tmp == 0x9)) {

            queue_sw_to_pm3(procedureByte);
            
            // After valid SW, quite loop
            return;
        }

        // ACK 10.3.3 
        if (procedureByte == ins) {

            // Send rest of the bytes to SIM card
            for(; si < curr_sim_len; si++) {
                Send_Data_To_UART_parity(to_sim[si]);
            }
            
            // read the rest of the bytes from SIM card
//            for(pi = 0; Receive_Data_From_UART0_parity_with_timeout(to_pm3 + PM3_CMD_HEADER_LEN + pi) && (pi < sizeof(to_pm3)); pi++);
            for(pi = 0; Receive_Data_From_UART0_parity_with_timeout(&to_pm3[PM3_CMD_HEADER_LEN + pi]); pi++);
            queue_pm3(pi);
            return;
        }
    } // end for   
}



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

// Interrupt Handler IRQ 6 
void I2C_ISR(void) interrupt 6
{
    switch(I2STAT) {
        case I2C_BUS_ERR: {
            // recover from bus error
            set_STO;
            break;
        }
    
        // 15.3.3 - Slave Receiver Mode

        // ================ SLAVE RECEIVE below =================
        // PM3 sending to 8051

        // Own SLA+W has been received - ACK has been transmitted		
        // Slave Recieve Address ACK
        case I2C_GC_RECEIVE_ACK:            
        case I2C_RECEIVE_ACK: {
            curr_cmd = CMD_POINTER_ZERO;
            curr_sim_len = 0;
            set_AA;
            break;
        }
		
		// Arbitration lost and own SLA+W has been received - ACK has been transmitted
        // Slave Receive Arbitration lost
        case I2C_GC_RECEIVE_ABR_ERR:        
        case I2C_RECEIVE_ABR_ERR: {
            clr_AA;
            set_STA;
            break;
        }
    
        // Data byte has been received - ACK has been transmitted
        // Slave Receive Data ACK
        case I2C_GC_RECEIVE_DATA_OK:        
        case I2C_RECEIVE_DATA_OK: {

            if (curr_cmd == CMD_POINTER_ZERO) {
                curr_cmd = I2DAT;
                set_AA;
                break;
            }

            // receive a byte from PROXMARK3.
            to_sim[curr_sim_len] = I2DAT;
            curr_sim_len++;
            
            if (curr_sim_len == TRANSFER_BUF_SIZE) {
                clr_AA;                
            } else {            
                set_AA;
            }
            break;
        }

        // Data byte has been received - NACK has been transmitted
        // Slave Receive Data NACK
        case I2C_GC_RECEIVE_DATA_ERR:
        case I2C_RECEIVE_DATA_ERR: {
            curr_sim_len = 0;
            set_AA;
            break;
		}

        // 15.3.3 - Slave Receiver Mode & 15.3.4 - Slave Transmitter Mode

        // ================ SLAVE TRANSMIT below =================
        // 8051 sending to PM3 

		// A STOP or repeated START has been received
        // Slave Transmit Repeat Start or Stop
        case I2C_STOP_START: {
            set_AA;

			switch(curr_cmd) {
                case I2C_DEVICE_CMD_GETVERSION: {
                    pCmdFunc = GETVERSION;
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
                case I2C_DEVICE_CMD_READ: {
                     break;
                }
                default: { 
                     break;
                }
            }

            curr_cmd = CMD_POINTER_ZERO;
            break; 
		}

        /*
        * 15.3.4 - Slave TRANSMIT Mode
        *
        * 8051 sending to PM3
        */
        
        // Own SLA+R has been received - ACK has been transmitted        
        // Slave Transmit Address ACK
        case I2C_TRANSMIT_ADR_OK: {
            curr_send_idx = 0;
            I2DAT = to_pm3[curr_send_idx];
            curr_send_idx++;
            set_AA;
            break;
        }

        // Arbitration lost and own SLA+R has been received - ACK has been transmitted
        // Slave Transmit Arbitration Lost
        case I2C_TRANSMIT_ABR_ERR: {
            clr_AA;
            set_STA;
            break;
        }

        // Data byte has been transmitted - ACK has been received
        // Slave Transmit Data ACK
        case I2C_TRANSMIT_DATA_OK: {

            // to_pm3_len    == how many bytes we have available to send to PM3.
            // curr_pm3_idx  == which byte are we sending
            I2DAT = to_pm3[curr_send_idx++];
            set_AA; 

            // last byte sent indicator
            /*
            if (curr_send_idx == to_pm3_len - 1) {
                clr_AA;
            } else {
                set_AA;
            }
            */
            break; 
		}

        // Data byte has been transmitted - NACK has been received
        // Slave Transmit Data NACK
        case I2C_TRANSMIT_DATA_ERR: {
            set_AA;
            break;
		}

        // Last Data byte has been transmitted - ACK has been received
        // Slave Transmit Last Data ACK
        case I2C_TRANSMIT_STOP: {
            set_AA;
            break;
		}
    }

    // SI should be last command of I2C ISR
    clr_SI;
    
    // Wait for STOP transmitted or bus error
    // free, STO is cleared by hardware
    while(STO);

} // end I2C_ISR