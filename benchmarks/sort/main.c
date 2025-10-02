long time();
int printf(const char*, ...);

#include <stdint.h>

#include "dataset.h"

#define RUN_TEST(name, id, fn) \
	gen_testdata(list); \
	printf("start %s sort\t\t... ", name); \
	times[id] = time(); \
	fn(list); \
	times[id] = time() - times[id]; \
	printf("%d cycles\n", times[id]);

uint16_t list[PROBLEM_SIZE];

enum {
	BUBBLE = 0,
	QUICK  = 1,
	MERGE  = 2
};

int check_sorted(uint16_t*);
void gen_testdata(uint16_t*);

void bubblesort(uint16_t*);
void quicksort(uint16_t*);
void mergesort(uint16_t*);

int main(void) {
	int times[3];

	printf("\nStarting list sorting\n");
	printf(  "---------------------\n");

	RUN_TEST("bubble", BUBBLE, bubblesort)
	RUN_TEST("quick",  QUICK,  quicksort)
	RUN_TEST("merge",  MERGE,  mergesort)

	printf("CSV#%d,%d,%d", times[BUBBLE], times[QUICK], times[MERGE]);

	return 0;
}

int check_sorted(uint16_t* l) {
	for (int ii = 0; ii < PROBLEM_SIZE; ++ii) {
		if (l[ii] != ii) return 0;
	}
	return 1;
}

void gen_testdata(uint16_t* l) {
	int a = 37;
	int c = 3;
	int m = PROBLEM_SIZE;

	int x = SEED;

	for (int ii = 0; ii < PROBLEM_SIZE; ++ii) {
		l[ii] = x;
		x = (a * x + c) % m;
	}
}

void bubblesort(uint16_t* l) {
	int ii = 0;
	while (ii < PROBLEM_SIZE - 1) {
		if (l[ii] > l[ii + 1]) {
			uint16_t t = l[ii + 1];
			l[ii + 1] = l[ii];
			l[ii] = t;
			if (ii != 0) --ii;
		} else ++ii;
	}
}

void quicksort_alg(uint16_t* l, unsigned int start, unsigned int end) {
	if (start >= end) return;

	int p = l[start];
	int lower  = start - 1;
	int higher = end + 1;

	while (1) {
		while (l[++lower]  > p);
		while (l[--higher] < p);

		if (lower >= higher) break;

		uint16_t t = l[lower];
		l[lower]  = l[higher];
		l[higher] = t;
	}

	// lower holds position of pivot
	
	{
		uint16_t t = l[lower];
		l[lower] = l[start];
		l[start] = t;
	}

	quicksort_alg(l, start,     lower - 1);
	quicksort_alg(l, lower + 1, end);
}

void quicksort(uint16_t* l) {
	quicksort_alg(l, 0, PROBLEM_SIZE - 1);
}

void mergesort_alg(uint16_t* l, unsigned int start, unsigned int end) {
	if (start < end) {
		unsigned int m = start + (end - start) / 2;

		mergesort_alg(l, start, m);
		mergesort_alg(l, m + 1, end);

		{
			int left = m + 1;

			if (l[m] <= l[left]) return;

			while (start <= m && left <= end) {
				if (l[start] <= l[left]) ++start;
				else {
					int v = l[left];
					int i = left;

					while (i != start) {
						l[i] = l[i - 1];
						--i;
					}
					l[start] = v;

					++start;
					++m;
					++left;
				}
			}
		}
	}
}

void mergesort(uint16_t* l) {
	mergesort_alg(l, 0, PROBLEM_SIZE - 1);
}
