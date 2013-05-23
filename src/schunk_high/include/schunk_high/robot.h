#ifndef ROBOT_HEADER_GUARD
#define ROBOT_HEADER_GUARD

#ifdef __cplusplus
extern "C" {
#endif


#include"link.h"
#include"gmatrix.h"
/**
The Structure holding the relevant info of a Robot.
*/
typedef struct Robot{
	int dofs;
	Link* fl;
} Robot;

extern Robot* Robot_create(void);
extern void Robot_free(Robot* r);
extern void Robot_addLink(Robot* r,double theta, double d, double a, double alpha);
extern DQ* Robot_FKM(Robot*r);
extern Link* Robot_getLink(Robot* r, int i);
PGMATRIX Robot_getAJacobian(Robot* r);
PGMATRIX Robot_getGJacobian(Robot* r);
PGMATRIX Robot_getOJacobianAngleP(Robot* r);
int Robot_updateLinksThetasByMatrix(Robot*r, PGMATRIX thetas);

#ifdef __cplusplus
}
#endif

#endif