#include "sort.h"

#include "dataset.h"

void sort_shuffle(num_t* list) {
	int a = 37;
	int c = 3;
	int m = MAT_SIZE * MAT_SIZE * 2;

	int x = SEED;

	for (int ii = 0; ii < m; ++ii) {
		list[ii] = x;
		x = (a * x + c) % m;
	}
}

void ms_merge(num_t* list, num_t* temp, int start, int mid, int end) {
	for (int ii = 0; ii < (mid - start); ++ii)
		temp[ii] = list[ii];
	
	num_t* left  = temp;
	num_t* right = list + mid;
	num_t* res   = list + start;

	num_t* left_lim  = temp  + (mid - start);
	num_t* right_lim = right + (end - mid);

	while (left < left_lim && right < right_lim) {
		if (*left <= *right)
			*res++ = *left++;
		else
			*res++ = *right++;
	}

	while (left < left_lim)
		*res++ = *left++;
	while (right < right_lim)
		*res++ = *right++;
}

void mergesort(num_t* list, int start, int end, num_t* temp) {
	if (end - start <= 1) return;

	int mid = (start + end) / 2;
	mergesort(list, start, mid - 1, temp);
	mergesort(list, mid, end, temp);

	ms_merge(list, temp, start, mid, end);
}

int qs_div(num_t* list, int l, int r) {
	int ii = l;
	int jj = r - 1;
	int piv = list[r];

	while (ii < jj) {
		while (ii < jj && list[ii] <= piv)
			++ii;
		while (jj > ii && list[jj] > piv)
			--jj;
		if (list[ii] > list[jj]) {
			num_t temp = list[ii];
			list[ii] = list[jj];
			list[jj] = temp;
		}
	}

	if (list[ii] > piv) {
		num_t temp = list[ii];
		list[ii] = list[r];
		list[r] = temp;
	} else
		ii = r;
	return ii;
}

void quicksort(num_t* list, int l, int r) {
	while (r > l) {
		int div = qs_div(list, l, r);
		if (r - div > div - l) {
			quicksort(list, l, div - 1);
			l = div + 1;
		} else {
			quicksort(list, div + 1, r);
			r = div - 1;
		}
	}
}

void hs_siftdown(num_t* list, int root, int end) {
	while (2 * root + 1 < end) {
		int child = 2 * root + 1;
		if (child + 1 < end && list[child] < list[child + 1])
			++child;
		if (list[root] < list[child]) {
			num_t temp = list[root];
			list[root] = list[child];
			list[child] = list[root];
			
			root = child;
		} else return;
	}
}

void hs_heapify(num_t* list, int len) {
	int start = (len - 2) / 2 + 1;

	while (start > 0) {
		start = start - 1;
		hs_siftdown(list, start, len);
	}
}

void heapsort(num_t* list, int len) {
	hs_heapify(list, len);

	while (len > 1) {
		--len;

		num_t temp = list[len];
		list[len] = list[0];
		list[0] = temp;

		hs_siftdown(list, 0, len);
	}
}

void sort_quick(num_t* list) {
	quicksort(list, 0, MAT_SIZE * MAT_SIZE * 2 - 1);
}

void sort_merge(num_t* list) {
	mergesort(list, 0, MAT_SIZE * MAT_SIZE * 2 - 1, list + MAT_SIZE * MAT_SIZE * 2);
}

void sort_bubble(num_t* list) {
	int n = MAT_SIZE * MAT_SIZE * 2;
	int s;
	do {
		s = 0;
		for (int ii = 0; ii < n - 1; ++ii) {
			if (list[ii] > list[ii + 1]) {
				num_t temp = list[ii];
				list[ii] = list[ii + 1];
				list[ii + 1] = temp;

				s = 1;
			}
		}
	} while (s);
}

void sort_heap(num_t * list) {
	heapsort(list, MAT_SIZE * MAT_SIZE * 2);
}
