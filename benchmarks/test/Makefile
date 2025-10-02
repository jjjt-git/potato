COMMON_FLAGS := -mabi=ilp32 -march=rv32i_zicsr

INCLUDES := -I../../potato -I../../potato/libsoc

LDSCRIPT := ../../potato/software/potato.ld
STARTS   := ../../potato/software/start.S

CFLAGS  := $(COMMON_FLAGS) $(INCLUDES) -MD -Os -DTIME -DRISCV -DUSE_MYSTDLIB -ffreestanding -nostdlib -Wno-implicit-int -Wno-implicit-function-declaration -fdata-sections -ffunction-sections -fanalyzer -fno-builtin
LDFLAGS := $(COMMON_FLAGS) -Wl,-Bstatic,-T,$(LDSCRIPT),-Map,matmul.map,--strip-debug -static-libgcc -lgcc -nostartfiles -Wl,-m,elf32lriscv --specs=nosys.specs -Wl,--no-relax,--gc-sections
OBJS    := $(addprefix obj/,main.o start.o stdlib.o trace.o handler.o)

TC := riscv-unknown-elf-

all: ../test.bin ../test.elf

../test.elf: $(OBJS) $(LDSCRIPT)
	$(TC)gcc $(LDFLAGS) $(OBJS) -o $@
	chmod -x $@

obj/%.o: %.c $(MAKEFILE_LIST) | obj/
	$(TC)gcc -c $(CFLAGS) $< -o $@

obj/start.o: $(STARTS) | obj
	$(TC)gcc -c $(CFLAGS) $< -o $@

obj/trace.o obj/handler.o obj/stdlib.o: obj/%.o: ../%.c | obj
	$(TC)gcc -c $(CFLAGS) $< -o $@

obj/:
	@mkdir -p $@

clean clear:
	rm -rf obj/ ../test.elf test.map ../test.bin

../%.bin: ../%.elf
	$(TC)objcopy -j .text -j .data -j .rodata -O binary $< $@

.PHONY: clean clear
