#ifndef LINK_HEADER_GUARD
#define LINK_HEADER_GUARD

#ifdef __cplusplus
extern "C" {
#endif

#include"dualquaternion.h"

typedef struct Link{
	double theta;
	double d;
	double a;
	double alpha;
	double jvel;
	//Next
	struct Link* n;
} Link;

extern DQ* Link_DQFT(Link* l);

extern struct Link* Link_create(double theta, double d, double a, double alpha);

extern void Link_free(Link* l);

#ifdef __cplusplus
}
#endif

#endif