BMARKS := dhry coremark matmul test

.PHONY: all clean

all: $(addsuffix .coe,$(BMARKS)) $(addsuffix .mem,$(BMARKS))

clear:
	rm -f coremark.elf
	rm -f *.bin *.mem *.coe *.hex

$(addsuffix .bin,$(BMARKS)): %.bin: %.elf
	riscv-unknown-elf-objcopy -j .text -j .data -j .rodata -O binary $< $@

$(addsuffix .mem,$(BMARKS)): %.mem: %.hex
	echo "@0000" > $@
	cat $< >> $@

$(addsuffix .coe,$(BMARKS)): %.coe: %.hex
	echo "memory_initialization_radix=16;" > $@
	echo "memory_initialization_vector=" >> $@
	cat $< >> $@
	echo ";" >> $@

$(addsuffix .hex,$(BMARKS)): %.hex: %.bin
	hexdump -v -e '1/4 "%08x\n"' $< > $@

coremark.elf: coremark/coremark.bin

coremark.elf:
	cp $< $@
