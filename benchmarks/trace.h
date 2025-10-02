#pragma once

void dump_trace(void);
void start_cr_trace(void);
void stop_cr_trace(void);

#if defined(__riscv) || defined(__riscv32) || defined(__riscv__) || defined(__riscv)
#define get_miss_count(v1, v2) \
	asm volatile ("nop":::); \
	asm volatile ("nop":::); \
	asm volatile ("nop":::); \
	asm volatile ("nop":::); \
	asm volatile ("nop":::); \
	asm volatile ("nop":::); \
	asm volatile ("csrr %0, %1" : "=r" (v1) : "i" (0xBFE) : ); \
	asm volatile ("csrr %0, %1" : "=r" (v2) : "i" (0xBFD) : )
#define set_miss_count(v1, v2) \
	asm volatile ("nop":::); \
	asm volatile ("nop":::); \
	asm volatile ("nop":::); \
	asm volatile ("nop":::); \
	asm volatile ("nop":::); \
	asm volatile ("nop":::); \
	asm volatile ("csrw %1, %0" : : "r" (v1), "i" (0xBFE) : ); \
	asm volatile ("csrw %1, %0" : : "r" (v2), "i" (0xBFD) : )
#define get_cache_config(v) \
	asm volatile ("csrr %0, %1" : "=r" (v) : "i" (0xBFF) : )
#else
#define get_miss_count(v1, v2) \
	v1 = 0; \
	v2 = 0
#define set_miss_count(v1, v2)
#define get_cache_config(v) v = 0
#endif
