# Host build of the protocol layers, so the T=0 and T=1 state machines can be
# exercised without a Proxmark3, a SIM module or a smart card in the loop.
#
#   make        stage, build and run every check
#   make clean
#
# The 8051 specific parts (SFRs, timers, UART) are replaced by the mocks in
# mock.c / mock_t0.c, which also carry small card models to talk to.  The
# firmware sources themselves are compiled unmodified apart from main() and the
# interrupt handler being taken out of SIM.C.

CC     ?= gcc
B       = build
SRC     = ..
CFLAGS  = -std=gnu89 -Wall -Wextra -Wno-unused-parameter -Wno-main -include pre.h -I$(B) -I.

TESTS = $(B)/t1test $(B)/t0test $(B)/e2etest $(B)/atrtest $(B)/i2ctest $(B)/uarttest $(B)/clocktest

# staging has to finish before make can match the object pattern rules, so the
# build runs as a second pass
all:
	@$(MAKE) --no-print-directory $(STAMP)
	@$(MAKE) --no-print-directory run

run: $(TESTS)
	@rc=0; for t in $(TESTS); do echo "=== $$t ==="; ./$$t || rc=1; done; exit $$rc

# ---- stage the firmware sources next to the host stand ins ----
STAMP = $(B)/.staged

$(STAMP): $(SRC)/N76E003.h $(SRC)/Common.h $(SRC)/Function_Define.h $(SRC)/SIM.C \
          $(SRC)/t1.c $(SRC)/iso7816.c $(SRC)/uart.c $(SRC)/timer.c \
          mock.c mock_t0.c pre.h Makefile $(wildcard $(SRC)/*.h) $(wildcard test_*.c)
	@mkdir -p $(B)
	@cp $(SRC)/*.c $(SRC)/*.h $(B)/
	@sed -e 's/interrupt 4 using 1//' -e 's/while (LOAD);/LOAD = 0;/' $(SRC)/uart.c > $(B)/uart.c
	@cp $(SRC)/Function_Define.h $(B)/Function_define.h
	@echo '/* stub */' > $(B)/intrins.h
	@sed -e 's/^typedef bit  *BIT;/typedef unsigned char BIT;/' \
	     -e 's/^extern bit BIT_TMP;/extern unsigned char BIT_TMP;/' \
	     $(SRC)/Common.h > $(B)/Common.h
	@sed -e 's/\binterrupt  *6\b//' $(SRC)/SIM.C > $(B)/sim_main.c
	@python3 -c "import re; s=open('$(SRC)/N76E003.h').read(); \
names=sorted(set(re.findall(r'^\s*(?:sfr16|sfr|sbit)\s+([A-Za-z_]\w*)', s, re.M))); \
open('$(B)/N76E003.h','w').write('#ifndef FAKE_N76\n#define FAKE_N76\n' \
 + ''.join('extern volatile unsigned char %s;\n' % n for n in names) \
 + ''.join('#define SET_BIT%d 0x%02X\n' % (i,1<<i) for i in range(8)) + '#endif\n'); \
open('$(B)/sfrdefs.c','w').write('/* host stand ins for the N76E003 SFRs */\n' \
 + ''.join('volatile unsigned char %s;\n' % n for n in names))"
	@cp mock.c mock_t0.c test_*.c $(B)/
	@touch $@

$(B)/%.o: $(B)/%.c $(STAMP)
	$(CC) $(CFLAGS) -c -o $@ $<

$(B)/sim_main.o: $(B)/sim_main.c $(STAMP)
	$(CC) $(CFLAGS) -Dstatic= -Dmain=fw_main_unused -c -o $@ $<

$(B)/t1test:  $(B)/t1.o $(B)/iso7816.o $(B)/mock.o $(B)/test_t1.o
	$(CC) -o $@ $^
$(B)/t0test:  $(B)/sim_main.o $(B)/t1.o $(B)/iso7816.o $(B)/mock_t0.o $(B)/sfrdefs.o $(B)/test_t0.o
	$(CC) -o $@ $^
$(B)/e2etest: $(B)/sim_main.o $(B)/t1.o $(B)/iso7816.o $(B)/mock.o $(B)/sfrdefs.o $(B)/test_e2e.o
	$(CC) -o $@ $^
$(B)/atrtest: $(B)/iso7816.o $(B)/uart.o $(B)/sfrdefs.o $(B)/test_atr.o
	$(CC) -o $@ $^
$(B)/i2ctest: $(B)/sim_main.o $(B)/t1.o $(B)/iso7816.o $(B)/mock.o $(B)/sfrdefs.o $(B)/test_i2c.o
	$(CC) -o $@ $^
$(B)/uarttest: $(B)/uart.o $(B)/sfrdefs.o $(B)/test_uart.o
	$(CC) -o $@ $^
$(B)/clocktest: $(B)/uart.o $(B)/timer.o $(B)/iso7816.o $(B)/sfrdefs.o $(B)/test_clock.o
	$(CC) -o $@ $^

clean:
	rm -rf $(B)

.PHONY: all run clean
