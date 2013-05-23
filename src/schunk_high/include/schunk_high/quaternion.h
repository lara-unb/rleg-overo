#ifndef QUATERNION_HEADER_GUARD
#define QUATERNION_HEADER_GUARD

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Q{
	double v[4];
}Q;

typedef Q Quaternion;

extern Q* Q_alloc();

extern Q* Q_create(double a, double b, double c, double d);

extern Q* Q_copy(Q* q);

extern void Q_free(Q* q);

extern void Q_print(Q* q);

extern Q* Q_add(Q* a, Q* b, int freeA, int freeB);

extern Q* Q_mult(Q* a, Q* b, int freeA, int freeB);

extern Q* Q_multd(double a, Q*b, int freeB);

extern Q* Q_getConj(Q* a, int freeA);

extern Q* Q_sub(Q* a, Q* b, int freeA, int freeB);

extern double Q_getPitchAngle(Q* q);

#ifdef __cplusplus
}
#endif

#endif
