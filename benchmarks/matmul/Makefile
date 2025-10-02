COMMON_FLAGS := -mabi=ilp32 -march=rv32i_zicsr

MAT_SIZE := 64

X86FLAGS := -ggdb -DMAT_SIZE=$(MAT_SIZE) -Os

INCLUDES := -I../../potato -I../../potato/libsoc

LDSCRIPT := ../../potato/software/potato.ld
STARTS   := ../../potato/software/start.S

CFLAGS  := -DMAT_SIZE=$(MAT_SIZE) $(COMMON_FLAGS) $(INCLUDES) -MD -Os -DTIME -DRISCV -DUSE_MYSTDLIB -ffreestanding -nostdlib -Wno-implicit-int -Wno-implicit-function-declaration -fdata-sections -ffunction-sections -fanalyzer -fno-builtin
LDFLAGS := $(COMMON_FLAGS) -Wl,-Bstatic,-T,$(LDSCRIPT),-Map,matmul.map,--strip-debug -static-libgcc -lgcc -nostartfiles -Wl,-m,elf32lriscv --specs=nosys.specs -Wl,--no-relax,--gc-sections
OBJS    := $(addprefix obj/,main.o matrix.o stdlib.o start.o handler.o dataset.o trace.o sort.o)

X86SRCS := main.c matrix.c dataset.c sort.c ../trace.x86.c

TC := riscv-unknown-elf-

all: ../matmul.bin ../matmul.elf matmul.debug

dataset.c: matmul_gendata.pl $(MAKEFILE_LIST)
	./matmul_gendata.pl --size $(MAT_SIZE) > $@

dataset.h: sort_genseed.pl $(MAKEFILE_LIST)
	./$< > $@

../matmul.elf: $(OBJS) $(LDSCRIPT)
	$(TC)gcc $(LDFLAGS) $(OBJS) -o $@
	chmod -x $@

matmul.debug: $(X86SRCS)
	gcc $(X86FLAGS) $^ -o $@

obj/%.o: %.c $(MAKEFILE_LIST) | obj/
	$(TC)gcc -c $(CFLAGS) $< -o $@

matmul.debug obj/sort.o: dataset.h

obj/start.o: $(STARTS) | obj
	$(TC)gcc -c $(CFLAGS) $< -o $@

obj/trace.o obj/handler.o obj/stdlib.o: obj/%.o: ../%.c | obj/
	$(TC)gcc -c $(CFLAGS) $< -o $@

obj/:
	@mkdir -p $@

clean clear:
	rm -rf obj/ ../matmul.elf matmul.map ../matmul.bin matmul.debug
	rm dataset.c

../%.bin: ../%.elf
	$(TC)objcopy -j .text -j .data -j .rodata -O binary $< $@

.PHONY: clean clear
