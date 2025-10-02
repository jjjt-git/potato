#include"matrix.h"

void clear_mat(mat_t trg) {
	for (int ii = 0; ii < MAT_SIZE; ++ii)
		for (int jj = 0; jj < MAT_SIZE; ++jj)
			trg[ii * MAT_SIZE + jj] = 0;
}

void mm_naive_ijk(mat_t A, mat_t B, mat_t C) {
	for (int ii = 0; ii < MAT_SIZE; ++ii) {
		for (int jj = 0; jj < MAT_SIZE; ++jj) {
			num_t sum = 0;
			for (int kk = 0; kk < MAT_SIZE; ++kk)
				sum += MUL(A[ii * MAT_SIZE + kk], B[kk * MAT_SIZE + jj]);
			C[ii * MAT_SIZE + jj] = sum;
		}
	}
}

void mm_naive_kij(mat_t A, mat_t B, mat_t C) {
	for (int kk = 0; kk < MAT_SIZE; ++kk) {
		for (int ii = 0; ii < MAT_SIZE; ++ii) {
			num_t Aik = A[ii * MAT_SIZE + kk];
			for (int jj = 0; jj < MAT_SIZE; ++jj)
				C[ii * MAT_SIZE + jj] += MUL(Aik, B[kk * MAT_SIZE + jj]);
		}
	}
}

void mm_naive_jki(mat_t A, mat_t B, mat_t C) {
	for (int jj = 0; jj < MAT_SIZE; ++jj) {
		for (int kk = 0; kk < MAT_SIZE; ++kk) {
			num_t Bkj = B[kk * MAT_SIZE + jj];
			for (int ii = 0; ii < MAT_SIZE; ++ii)
				C[ii * MAT_SIZE + jj] += MUL(A[ii * MAT_SIZE + kk], Bkj);
		}
	}
}

void mm_blocked_ijk(mat_t A, mat_t B, mat_t C, unsigned int bsize) {
	for (int kkb = 0; kkb < MAT_SIZE; kkb += bsize) {
		for (int jjb = 0; jjb < MAT_SIZE; jjb += bsize) {
			for (int ii = 0; ii < MAT_SIZE; ++ii) {
				for (int jj = jjb; jj < jjb + bsize; ++jj) {
					num_t sum = 0;
					for (int kk = kkb; kk < kkb + bsize; ++kk)
						sum += MUL(A[ii * MAT_SIZE + kk], B[kk * MAT_SIZE + jj]);
					C[ii * MAT_SIZE + jj] = sum;
				}
			}
		}
	}
}

void mm_naive_ijk_T(mat_t A, mat_t B, mat_t C) {
	for (int ii = 0; ii < MAT_SIZE; ++ii) {
		for (int jj = 0; jj < MAT_SIZE; ++jj) {
			num_t sum = 0;
			for (int kk = 0; kk < MAT_SIZE; ++kk)
				sum += MUL(A[kk * MAT_SIZE + ii], B[jj * MAT_SIZE + kk]);
			C[jj * MAT_SIZE + ii] = sum;
		}
	}
}

void mm_naive_kij_T(mat_t A, mat_t B, mat_t C) {
	for (int kk = 0; kk < MAT_SIZE; ++kk) {
		for (int ii = 0; ii < MAT_SIZE; ++ii) {
			num_t Aik = A[kk * MAT_SIZE + ii];
			for (int jj = 0; jj < MAT_SIZE; ++jj)
				C[jj * MAT_SIZE + ii] += MUL(Aik, B[jj * MAT_SIZE + kk]);
		}
	}
}

void mm_naive_jki_T(mat_t A, mat_t B, mat_t C) {
	for (int jj = 0; jj < MAT_SIZE; ++jj) {
		for (int kk = 0; kk < MAT_SIZE; ++kk) {
			num_t Bkj = B[jj * MAT_SIZE + kk];
			for (int ii = 0; ii < MAT_SIZE; ++ii)
				C[jj * MAT_SIZE + ii] += MUL(A[kk * MAT_SIZE + ii], Bkj);
		}
	}
}

void mm_blocked_ijk_T(mat_t A, mat_t B, mat_t C, unsigned int bsize) {
	for (int kkb = 0; kkb < MAT_SIZE; kkb += bsize) {
		for (int jjb = 0; jjb < MAT_SIZE; jjb += bsize) {
			for (int ii = 0; ii < MAT_SIZE; ++ii) {
				for (int jj = jjb; jj < jjb + bsize; ++jj) {
					num_t sum = 0;
					for (int kk = kkb; kk < kkb + bsize; ++kk)
						sum += MUL(A[kk * MAT_SIZE + ii], B[jj * MAT_SIZE + kk]);
					C[jj * MAT_SIZE + ii] = sum;
				}
			}
		}
	}
}
