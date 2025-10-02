#define NUM 4

#include"../trace.h"

int main(void) {
	volatile int test[NUM];

	for (int ii = 0; ii < NUM; ++ii)
		test[ii] = ii;

	for (int ii = 0; ii < NUM; ++ii)
		++test[ii];

	for (int ii = 0; ii < NUM / 2; ++ii) {
		int temp = test[ii + NUM / 2];
		test[ii + NUM / 2] = test[ii];
		test[ii] = test[ii + NUM / 2];
	}

	int t;
	get_cache_config(t);
	
	return 0;
}
