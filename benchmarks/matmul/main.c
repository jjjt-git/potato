#include<stdint.h>

uint64_t time();
int printf(const char*, ...);

#include "../trace.h"

#include "sort.h"
#include "matrix.h"

extern num_t DATA_REGION[3 * MAT_SIZE * MAT_SIZE];

enum {
	NAIVE_JKI = 0,
	NAIVE_KIJ = 1,
	NAIVE_IJK = 2,
	BLOCKED_IJK_4  = 3,
	BLOCKED_IJK_8  = 4,
	BLOCKED_IJK_16 = 5,
	MERGESORT  = 6,
	QUICKSORT  = 7,
	BUBBLESORT = 8,
	HEAPSORT   = 9,
	NAIVE_JKI_T = 10,
	NAIVE_KIJ_T = 11,
	NAIVE_IJK_T = 12,
	BLOCKED_IJK_4_T  = 13,
	BLOCKED_IJK_8_T  = 14,
	BLOCKED_IJK_16_T = 15
};

#define MATRIX_TEST(name, time_f, fun) \
	clear_mat(C); \
	printf("start %s\t\t\t\t... ", name); \
	start_cr_trace(); \
	set_miss_count(0, 0); \
	fun(A, B, C); \
	get_miss_count(times[time_f], times_r[time_f]); \
	stop_cr_trace(); \
	printf("%d/%d misses\n", times[time_f], times_r[time_f]); \
	dump_trace();

#define BMATRIX_TEST(name, time_f, fun, bsize) \
	clear_mat(C); \
	printf("start %s block size %d\t\t... ", name, bsize); \
	start_cr_trace(); \
	set_miss_count(0, 0); \
	fun(A, B, C, bsize); \
	get_miss_count(times[time_f], times_r[time_f]); \
	stop_cr_trace(); \
	printf("%d/%d misses\n", times[time_f], times_r[time_f]); \
	dump_trace();

#define SORT_TEST(name, time_f, fun) \
	sort_shuffle(L); \
	printf("start %s\t\t\t\t... ", name); \
	start_cr_trace(); \
	set_miss_count(0, 0); \
	fun(L); \
	get_miss_count(times[time_f], times_r[time_f]); \
	stop_cr_trace(); \
	printf("%d/%d misses\n", times[time_f], times_r[time_f]); \
	dump_trace();

#define BENCHMARK_CNT 16

int main(void) {
	unsigned int times[BENCHMARK_CNT], times_r[BENCHMARK_CNT];

	mat_t A = DATA_REGION;
	mat_t B = DATA_REGION + MAT_SIZE * MAT_SIZE;
	mat_t C = DATA_REGION + 2 * MAT_SIZE * MAT_SIZE;

	num_t* L = DATA_REGION;

	printf("\nstarting benchmarks\n");
	printf(  "-------------------\n");

	MATRIX_TEST("naive jki", NAIVE_JKI, mm_naive_jki)
	MATRIX_TEST("naive kij", NAIVE_KIJ, mm_naive_kij)
	MATRIX_TEST("naive ijk", NAIVE_IJK, mm_naive_ijk)

	BMATRIX_TEST("blocked ijk", BLOCKED_IJK_4, mm_blocked_ijk, 4)
	BMATRIX_TEST("blocked ijk", BLOCKED_IJK_8, mm_blocked_ijk, 8)
	BMATRIX_TEST("blocked ijk", BLOCKED_IJK_16, mm_blocked_ijk, 16)

	MATRIX_TEST("naive jki transposed", NAIVE_JKI_T, mm_naive_jki_T)
	MATRIX_TEST("naive kij transposed", NAIVE_KIJ_T, mm_naive_kij_T)
	MATRIX_TEST("naive ijk transposed", NAIVE_IJK_T, mm_naive_ijk_T)

	BMATRIX_TEST("blocked ijk transposed", BLOCKED_IJK_4_T, mm_blocked_ijk_T, 4)
	BMATRIX_TEST("blocked ijk transposed", BLOCKED_IJK_8_T, mm_blocked_ijk_T, 8)
	BMATRIX_TEST("blocked ijk transposed", BLOCKED_IJK_16_T, mm_blocked_ijk_T, 16)

	SORT_TEST("mergesort ", MERGESORT, sort_merge)
	SORT_TEST("quicksort ", QUICKSORT, sort_quick)
	SORT_TEST("bubblesort", BUBBLESORT, sort_bubble)
	SORT_TEST("heapsort  ", HEAPSORT, sort_heap)

	struct {
		unsigned int BLOCK_SIZE;
		unsigned int WAYNESS;
		unsigned int HISTORY_LEN;
		unsigned int CACHE_DEPTH;
		unsigned int HAS_RANDOM;
		unsigned int HAS_FIFO;
		unsigned int HAS_MRU;
		unsigned int HAS_DLFU;
		unsigned int HAS_LRU;
	} cc;

	{
		int temp;
		get_cache_config(temp);

		cc.HAS_LRU    = (temp >> 0) & ((1 << 1) - 1);
		cc.HAS_DLFU   = (temp >> 1) & ((1 << 1) - 1);
		cc.HAS_MRU    = (temp >> 2) & ((1 << 1) - 1);
		cc.HAS_FIFO   = (temp >> 3) & ((1 << 1) - 1);
		cc.HAS_RANDOM = (temp >> 4) & ((1 << 1) - 1);

		cc.CACHE_DEPTH = (temp >>  5) & ((1 << 11) - 1);
		cc.HISTORY_LEN = (temp >> 16) & ((1 <<  8) - 1);
		cc.WAYNESS     = (temp >> 24) & ((1 <<  4) - 1);
		cc.BLOCK_SIZE  = (temp >> 28) & ((1 <<  4) - 1);
	}

	printf("HEAD#SIZE,BLOCK_LEN,WAYS,HIST,RAND,FIFO,DLFU,LRU,MRU,MM1,MM1R,MM2,MM2R,MM3,MM3R,MMB4,MMB4R,MMB8,MMB8R,MMB16,MMB16R,MERGE,MERGE_R,QUICK,QUICK_R,BUBBLE,BUBBLE_R,HEAP,HEAP_R,MMT1,MMT1R,MMT2,MMT2R,MMT3,MMT3R,MMTB4,MMTB4R,MMTB8,MMTB8R,MMTB16,MMTB16R\n");

	printf("CSV#%d,%d,%d,%d,%d,%d,%d,%d,%d,",
		4 * cc.BLOCK_SIZE * cc.CACHE_DEPTH,
		cc.BLOCK_SIZE,
		cc.WAYNESS,
		cc.HISTORY_LEN,
		cc.HAS_RANDOM,
		cc.HAS_FIFO,
		cc.HAS_DLFU,
		cc.HAS_LRU,
		cc.HAS_MRU
	);

	for (int ii = 0; ii < BENCHMARK_CNT; ++ii)
		printf("%d,%d", times[ii], times_r[ii]);

	printf("\n");

	return 0;
}
