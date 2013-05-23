/**
DualQuaternion Library.

DualQuaternion Allocation, Creation and relevant Operations.
Uses the Quaternion Library.

@author Murilo Marques Marinho.
@see quaternion.c
*/

#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE 1
#endif

#include<schunk_high/dualquaternion.h>
#include<schunk_high/quaternion.h>

#include<stdio.h>
#include<stdlib.h>


/**
Allocates a Dual Quaternion. Does not initialize any of it's components.

@return A DualQuaternion* to a the newly allocated memory region.
*/
DQ* DQ_alloc(){
	return (DQ*)malloc(sizeof(DQ));
}

/**
Allocated and initializes a DualQuaternion.

@param a Primary part scalar part
@param b Primary part i part
@param c Primary part j part
@param d Primary part k part
@param e Dual part scalar part
@param f Dual part i part
@param g Dual part j part
@param h Dual part k part
@return A DualQuaternion = [a b c d e f g h]'
*/
DQ* DQ_create(double a, double b, double c, double d, double e, double f, double g, double h){
	DQ* dq = DQ_alloc();
	dq->p = Q_create(a,b,c,d);	
	dq->d = Q_create(e,f,g,h);
	return dq;
}

/**
Allocated and initializes a DualQuaternion.

@param a The quaternion representing the translation.
@param b The quaternion representing the rotation.
@return A DualQuaternion = []
*/
DQ* DQ_createFromQuaternion(Q* t, Q* r){
	DQ* dq = DQ_alloc();
	//Primary part is r
	dq->p = Q_copy(r);
	//Dual part is (1/2)*(t*r)
	Q* temp = Q_mult(t,r,FALSE,FALSE);
	dq->d = Q_multd(0.5,temp,TRUE);
	return dq;
}

/**
DualQuaternion Rotation.

@param dq A DualQuaternion.
@return a Unity Quaternion representing dq's Rotation.
*/
Q* DQ_getR(DQ* dq){
	Q* r = Q_copy(dq->p);
	return r;
}

/**
DualQuaternion Translation considering a translation followed by a rotation.

@param dq The DualQuaternion from which you want to get the translation.
@return A Quaternion representing the translation.
*/
Q* DQ_getT(DQ* dq){
	Q* t;

	//2*(dq->d*(dq->p)')
	t = Q_multd(2,Q_mult(dq->d,Q_getConj(dq->p,FALSE),FALSE,TRUE),TRUE);

	return t;
}


/**
Free's an Allocated DualQuaternion.

@param dq a DualQuaternion* to the Allocated region to be freed.
@return NULL
*/
DQ* DQ_free(DQ* dq){
	Q_free(dq->p);
	Q_free(dq->d);
	free(dq);
	return NULL;
}

/**
DualQuaternion Multiplication.

It uses the expanded H+ operator on the first term to make the multiplication.
Remember that DualQuaternion Multiplication is not commutative in general.

@param a The first DualQuaternion
@param b The second DualQuaternion
@param freeA TRUE if you want to free a from memory.
@param freeB TRUE if you want to free b from memory.
@returb c = a*b
*/
DQ* DQ_mult(DQ* a, DQ* b, int freeA, int freeB){

	//Alloc
	DQ* c = DQ_alloc();
	Q* temp1;
	Q* temp2;
	
	//Primary Part = P(a)*P(b).
	c->p = Q_mult(a->p,b->p,FALSE,FALSE);
	
	//Dual Part    = P(a)*D(b) + D(a)*P(b)
    temp1 = Q_mult(a->p,b->d,FALSE,FALSE);
	temp2 = Q_mult(a->d,b->p,FALSE,FALSE);
	c->d  = Q_add(temp1,temp2,TRUE,TRUE);

	if(freeA){DQ_free(a);}
	if(freeB){DQ_free(b);}
	
	return c;
}

/**
Gets the "PITCH" angle.

@param dq The DualQuaternion representing the rotation.
@return a Double with the desired value in radians.
*/
double DQ_getPitchAngle(DQ* dq){

	return Q_getPitchAngle(dq->p);
	
}


/**
Prints a DualQuaternion.

@param dq The DualQuaternion to be printed.
*/
void DQ_print(DQ* dq){
	printf("\t[%e %e %e %e  %e %e %e %e]\n",dq->p->v[0],dq->p->v[1],dq->p->v[2],dq->p->v[3],dq->d->v[0],dq->d->v[1],dq->d->v[2],dq->d->v[3]);
}


