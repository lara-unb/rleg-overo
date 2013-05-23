
#ifndef GMATRIX_PLUS_HEADER_GUARD

#define GMATRIX_PLUS_HEADER_GUARD


#include"gmatrix.h"
#include"gmatrix_linalg.h"
#include"dualquaternion.h"


#ifdef __cplusplus
extern "C" {
#endif

extern void PGMATRIX_SET_COLUMN_BY_DQ(PGMATRIX Pmat, int col, DQ* dq );

extern PGMATRIX PGMATRIX_CREATE_DIAG_FROM_ARRAY(int Dim, GMATRIX_FLOAT* Array);

extern PGMATRIX PGMATRIX_CREATE_FROM_ARRAY(int Nr, int Nc, GMATRIX_FLOAT* Array);

extern PGMATRIX PGMATRIX_CREATE_ZEROES(int Nr, int Nc);

extern PGMATRIX PGMATRIX_SUBTRACT_COPY_FREE(PGMATRIX pMatA, PGMATRIX pMatB, int freeA, int freeB);

extern PGMATRIX PGMATRIX_ADD_COPY_FREE(PGMATRIX pMatA, PGMATRIX pMatB, int freeA, int freeB);

extern PGMATRIX PGMATRIX_MULTIPLY_COPY_FREE(PGMATRIX pMatA, PGMATRIX pMatB, int freeA, int freeB);

extern PGMATRIX PGMATRIX_MULTIPLY_CONST_COPY_FREE(PGMATRIX pMat, GMATRIX_FLOAT a, int freeMat);

extern PGMATRIX PGMATRIX_PSEUDOINVERSE_PLUS(PGMATRIX pA);

extern PGMATRIX PGMATRIX_DAMPED_LEASTSQUARES_PSEUDOINVERSE(PGMATRIX pMat, double k);

extern PGMATRIX PGMATRIX_RIGHT_PSEUDOINVERSE(PGMATRIX pMat);

extern PGMATRIX PGMATRIX_RIGHT_PSEUDOINVERSE_NULLSPACE_PROJECTOR(PGMATRIX pMat);

#ifdef __cplusplus
}
#endif

#endif