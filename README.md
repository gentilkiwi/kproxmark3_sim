# kproxmark3_sim

A test project to handle SIM module of the Proxmark3, in C
Includes legacy `sim011.asm` and `sim013.asm` building

## Dependencies & Tools

- `sim011.asm` and `sim013.asm`, from Sentinel - original Proxmark3 - https://github.com/RfidResearchGroup/proxmark3/tree/master/tools/simmodule
- `ldrom.bin`, from @wh201906 - https://github.com/wh201906/PM3-Modding/tree/main/PM3_ISO7816
- `hex2bin`, from Jacques Pelletier - https://hex2bin.sourceforge.net/ & https://sourceforge.net/projects/hex2bin/

- Keil uVision v5,  the C51 development tools,  https://www.keil.com/download/


## How to compile

`F7` to compile the project and in the project folder there is a sub folder called "Objects".  
The artifact you want is `sim_c.bin`

### Keil needs a licence for this

The evaluation build of C51 stops at 2 KB of code:

    *** FATAL ERROR L250: CODE SIZE LIMIT IN RESTRICTED VERSION EXCEEDED
        LIMIT:   0800H BYTES

With T=1, ATR parsing and PPS the firmware is around 6 KB, so it will not link
under that cap.  This is a toolchain limit and not a hardware one - the
N76E003 has 18 KB of APROM, and the build uses 638 of the 768 bytes of XRAM.

Nuvoton hand out a free Keil C51 licence for their own 8051 parts; register it
through uVision under `File` -> `License Management`.  A stock evaluation
install will only ever build the older T=0 only firmware.


## How to install fw on Proxmark3 RDV4 sim module?

  - Take the `sim_c.bin` and copy to the proxmark3\client\resource folder.
  
  - Generate a sha512 file
  
  `sha512sum -b client/resources/sim_c.bin > client/resources/sim_c.sha512.txt`

  - Start the Proxmark3 client and run
    `smart upgrade -f sim_c.bin`


if all goes well,  you will have the following output
  
![Output of a successful run of the command Smart upgrade](/smart_upgrade_success.png)

  

## Protocol support

### v4.66: TA1=0x96 clock and receive fixes

TA1=0x96 selects F=512, D=32. At the default 4 MHz card clock this is
250 kbaud (4 us per ETU). The existing PPS command selects this rate with
payload `{0x00, 0x96}` for T=0 or `{0x01, 0x96}` for T=1, immediately after
reset/ATR. Check that its reply is `{1, requested_protocol, 0x96}` before
sending APDUs; advertising TA1 in the ATR alone does not select the rate.

The MCU now runs at 16 MHz while PWM1 supplies the existing 4 MHz card
clock on P1.1 (PIOCON0.PIO01). Edge-aligned PWM uses period 3 and duty 2.
Timer3 supplies UART0 baud: divisor 93 at the default rate, 8 at TA1=0x95,
and 4 at TA1=0x96. This avoids the N76E003's forbidden divisor 1 with SMOD=1.
The previous receive-buffer-only v4.66 build still used that invalid divisor.

Legacy SETBAUD values remain card-relative: TH1=0xFF translates to divisor 4
at the default card clock. SIM_CLC changes the card clock without slowing the
CPU; a clock change that would require divisor 1 is rejected. Timer0 uses
25 ms slices so a slice fits its 16-bit counter at the faster CPU clock.
Long guard and turnaround delays are split into bounded timer intervals.

UART0 now captures received bytes in an interrupt handler into a 64-byte
internal-RAM ring (63 usable slots). This covers processing gaps between
procedure bytes, block headers, payloads and checksums. Both protocol paths
consume the same queue. Queue overflow flags an invalid frame, using the
existing parity-error indication: T=0 returns an empty response and T=1 uses
its existing recovery path. T=0 also rejects received parity errors.

UART0 uses register bank 1 and high interrupt priority; transmission polls TI
with ES disabled and enables reception before returning. Keep bank 1 reserved
for UART0. Reception requires global interrupts enabled, as the normal I2C
initialization already does.

Validation: Keil C51 9.60.7 / BL51 6.22.4 LARGE, optimization 9 for size;
7939 bytes code, 645 bytes XDATA; the padded binary is 7940 bytes.
All seven host suites pass with ProxSpace MinGW GCC on Windows, including queue
wraparound/overflow, parity errors, T=0 error rejection and T=1 retransmission
for an error queued before reception starts. Host mocks do not model UART
timing or physical PWM output. The clock suite checks routing, baud divisors,
legacy restore commands and timer bounds. Clang, Linux and macOS were not tested.

Hardware validation reported on 2026-09-05: the same T=1 APDU returned an
identical response and 9000 at TA1=0x95 and 0x96.

Remaining hardware validation: exercise both protocols with a
card that accepts PPS1=0x96: long T=0 reads and writes, T=1 chained responses
and commands, LRC/CRC where supported, repeated APDUs, reset and renegotiation.
Check the card clock and 4 us ETU with a logic analyzer, compare complete
response bytes against the same commands at the default rate and at 0x95,
and exercise parity-error recovery. The host tests cannot establish that
the interrupt and main loop meet every real card's timing requirements.

The firmware talks to the card over UART0 in 9-bit mode, using TB8/RB8 as the
ISO/IEC 7816-3 even parity bit.  Both byte oriented (T=0) and block oriented
(T=1) transmission protocols are handled inside the module, so the Proxmark3
only ever hands over a plain APDU and gets a plain APDU back.

### I2C command set (address `0xC0`)

| Cmd    | Name           | Payload                | Reply                          |
|--------|----------------|------------------------|--------------------------------|
| `0x01` | `GENERATE_ATR` | -                      | ATR (also parsed by the module) |
| `0x02` | `SEND`         | raw bytes              | raw bytes from the card         |
| `0x03` | `READ`         | -                      | the last reply                  |
| `0x04` | `SETBAUD`      | TH1 reload value       | -                               |
| `0x05` | `SIM_CLC`      | CKDIV value            | -                               |
| `0x06` | `GETVERSION`   | -                      | major, minor                    |
| `0x07` | `SEND_T0`      | APDU                   | response APDU                   |
| `0x08` | `SEND_T1`      | APDU                   | response APDU                   |
| `0x09` | `PPS`          | protocol[, TA1]        | ok, active protocol, TA1        |

`0x08` and `0x09` are new in v4.51; `0x04` and `0x05` were declared but never
implemented in the C firmware before it. v4.66 translates their legacy values
to Timer3 and the independent card clock. A host can identify the firmware
version through `GETVERSION`.

### T=1

`SEND_T1` (`0x08`) runs the whole block layer in the module:

- chaining in both directions, honouring IFSC from the ATR and announcing
  IFSD = 254 with an `S(IFS)` request on the first exchange
- `R` block error recovery on parity errors, bad checksums and lost blocks,
  falling back to `S(RESYNCH)` and then reporting failure
- `S(WTX)` time extensions, `S(IFS)`, `S(ABORT)`, `S(RESYNCH)`
- LRC and CRC checksums, picked up from `TC(i)` of the ATR
- an automatic PPS to T=1 when the ATR offers T=1 but names T=0 first

Waiting times are no longer a flat 100 ms.  `GENERATE_ATR` parses the ATR and
derives WWT, CWT and BWT from `TC2` and `TB(i)`, so a card that takes its full
default block waiting time (~1.4 s) is no longer cut off.

Two things worth knowing on the Proxmark3 side:

- `SEND_T1` needs a host that knows to send `0x08`.  Until then a T=1 card can
  still be driven through the raw `SEND` path, which now waits BWT for the
  first byte and reads the block length driven instead of on a timeout.
- Anything that talks to the card must be issued as a write ending in STOP,
  followed by a separate `READ` - which is what `I2C_BufferWrite` plus
  `sc_rx_bytes` already do.  A combined write/read transaction only tolerates
  about 100 ms of clock stretching (see below), and `GENERATE_ATR`, `SEND*` and
  `PPS` can all hold the bus far longer than that.  `GETVERSION` is the only
  command answered inside a single transaction, and it never touches the card.

### I2C, and how long the module may hold the bus

The module signals "busy" by holding SCL low, and the master's patience for
that is not one number.  With `I2C_DELAY_1CLK` at `SpinDelayUsPrecision(20)`
the current `armsrc/i2c.c` gives:

| Where | Function | Cap |
|-------|----------|-----|
| Inside a byte, mid transaction | `WaitSCL_H()` | 5000 x 20 us = ~100 ms |
| Between a command and its reply | `WaitSCL_H_delay(SIM_WAIT_DELAY)` | 150000 x 20 us = ~3 s |

The 3 s figure is comfortably above a default T=1 block waiting time (~1.4 s),
so a slow card does not need `SC_WAIT`.  The 100 ms one is why a slow command
must never be dispatched from a repeated START.

### I2C slave fixes

- **The slave could latch itself into being a master.**  States `0x68`, `0x78`
  and `0xB0` (arbitration lost) set `STA`, which is the right answer for a
  device that was trying to be a master and lost - but this one never is.  With
  `STA` set the peripheral grabs the bus and emits a START of its own as soon
  as the line goes idle, in the middle of the Proxmark3's next transfer.
  Nothing ever cleared it again, so a single glitch was permanent until the
  next hardware reset.  `STA` and `STO` are now cleared on every interrupt
  exit, the way `sim011.asm` / `sim013.asm` always did it.
- **The slave could go deaf to its own address.**  `AA` was cleared to mark the
  last byte of a reply and to refuse an over long write, but `AA` also gates
  address recognition; any path that ended while it was still clear left the
  module unaddressable.  `AA` is now always set on exit - the master decides
  how much it reads, and an over long write is caught with a flag.
- **Schmitt trigger inputs were enabled on the wrong pin.**  `set_P0SR_6` with
  a comment claiming P0.6 was SCL: on the N76E003 P0.6 is UART0 TXD, and the
  I2C pins are P1.3 (SCL) and P1.4 (SDA).  Both now get hysteresis, which
  matters when the master bit bangs the bus with ~20 us edges rising through
  pull ups.  RXD (P0.7) gets it too, for the card's open drain I/O line.
- **An over long write was dispatched as an empty command.**  State `0x88` set
  `curr_sim_len` to 0 and the following STOP still ran the handler.  The
  transfer is now flagged and answered with a well formed empty reply.
- **A command byte of `0x00` was read as "no command latched yet"**, so the
  next data byte became the command.  There is an explicit flag now.
- **A read before anything was queued clocked out stale buffer contents** and
  kept acknowledging.  `0xA8` now always presents a valid length header, and a
  read past the end of a reply gets `0x00` instead of a repeat of the last byte.
- **The I2C interrupt was enabled before `I2ADDR` was written.**
- **The ISR parked a function pointer for the main loop to call.**  Keil counts
  taking a function's address as a call from that point, so every handler ended
  up in the interrupt's call tree as well as main's, and anything reachable
  from both lost its overlaid locals with `L15: MULTIPLE CALL TO SEGMENT`.  On
  this part that is not a warning to wave through - it is the linker quietly
  sharing a local variable with something in the other tree.  The ISR now hands
  over a command number and the main loop switches on it.

### Fixes carried in the same change

- `GENERATE_ATR`, `SEND` and `SEND_T0` wrote two bytes past the end of
  `to_pm3` when a card sent a full frame (the bound was checked after the
  store, not before)
- a `0x60` NULL procedure byte in `SEND_T0` decremented the send index, so the
  next ACK resent P3 instead of the first data byte
- a single byte ACK past the end of the command read past `to_sim`
- a missing SW2 was reported to the host as if it had arrived
- `REN` was toggled off after every received character, which drops the next
  one when a card streams characters back to back
- the UART ran 3.1 % off the nominal 10753 baud; the divisor is now 1.1 % off


## Host side tests

The protocol layers and the I2C slave build and run on a normal machine
against mock cards and a mock master, so the state machines can be exercised
without hardware:

    cd sim_c/test && make

`test/mock.c` and `test/mock_t0.c` hold the card models; the firmware sources
are compiled unmodified apart from `main()` and the interrupt handler being
taken out of `SIM.C`.


## A note on XRAM

The N76E003 has 768 bytes of XDATA and `to_sim` plus `to_pm3` already take 540
of them, so the build is tight.  If the linker reports `DATA SEGMENT TOO LARGE`
after adding anything, the cheapest levers are `T1_IFSD_WANTED` in `t1.h`
(setting it to 32 removes the `S(IFS)` exchange) and dropping
`I2C_DEVICE_CMD_PPS` from the dispatch table in `SIM.C`.
