#pragma once

#include"types.h"

#define MUL(a, b) (a * b)

typedef num_t* mat_t;

// assuming square matrices

void clear_mat(mat_t trg);

void mm_naive_jki  (mat_t A, mat_t B, mat_t C);
void mm_naive_kij  (mat_t A, mat_t B, mat_t C);
void mm_naive_ijk  (mat_t A, mat_t B, mat_t C);
void mm_blocked_ijk(mat_t A, mat_t B, mat_t C, unsigned int bsize);

void mm_naive_jki_T  (mat_t A, mat_t B, mat_t C);
void mm_naive_kij_T  (mat_t A, mat_t B, mat_t C);
void mm_naive_ijk_T  (mat_t A, mat_t B, mat_t C);
void mm_blocked_ijk_T(mat_t A, mat_t B, mat_t C, unsigned int bsize);

