#include<stdio.h>

#include"trace.h"

void dump_trace(void) {
	printf("\nTRACE#");
	for (int ii = 0; ii < 1024; ++ii);
	*((int*) 0xc0006000) = 0;
	printf("\n");
}

void start_cr_trace(void) {
	*((int*) 0xc0006004) = 0;
}
void stop_cr_trace(void) {
	*((int*) 0xc0006008) = 0;
}
