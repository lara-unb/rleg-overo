/**
DualQuaternion library Header.

@author Murilo Marques Marinho.
*/

#ifndef DUALQUATERNION_HEADER_GUARD
#define DUALQUATERNION_HEADER_GUARD

#ifdef __cplusplus
extern "C" {
#endif


#include"quaternion.h"
#include"gmatrix.h"

/**
A DualQuaternion.
*/
typedef struct DQ{
	/**
	A Quaternion representing it's Primary Part.
	*/
	Q* p;
	/**
	Q Quaternion representing it's Dual Part.
	*/
	Q* d;
}DQ;

/**
Another name for a DualQuaternion.
*/
typedef DQ DualQuaternion;

extern DQ* DQ_alloc();

extern DQ* DQ_create(double a, double b, double c, double d, double e, double f, double g, double h);

extern DQ* DQ_createFromQuaternion(Q* t, Q* r);

extern Q* DQ_getR(DQ* dq);

extern Q* DQ_getT(DQ* dq);

extern DQ* DQ_free(DQ* dq);

extern DQ* DQ_mult(DQ* a, DQ* b, int freeA, int freeB);

extern void DQ_print(DQ* dq);

extern double DQ_getPitchAngle(DQ* dq);

extern void PGMATRIX_SET_COLUMN_BY_DQ(PGMATRIX Pmat, int col, DQ* dq );

#ifdef __cplusplus
}
#endif


#endif
