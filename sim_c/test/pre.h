/* Turns the Keil C51 memory space keywords into nothing so gcc can build the
 * protocol sources on the host.  Only the pure logic is exercised here; the
 * UART and the timers are replaced by the mocks. */
#define code
#define xdata
#define data
#define idata
#define bit unsigned char
