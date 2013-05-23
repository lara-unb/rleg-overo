/**
Robot's Link Library.

Holds the DH's parameters of each Link and the transformations of each Link.

@author Murilo Marques Marinho.
*/

#include<stdlib.h>
#include<stdio.h>

#include<schunk_high/link.h>
#include<schunk_high/dualquaternion.h>

#include<math.h>

/**
Allocates a new Link and initializes it using the given DH parameters.

@param theta The link's DH's theta
@param d     The Link's DH's d
@param a     The Link's DH's a
@param alpha The Link's DH's alpha
@return A Link with the given DH parameters.
*/
Link* Link_create(double theta, double d, double a, double alpha){
	
	Link* l = (Link*)malloc(sizeof(Link));
	l->theta = theta;
	l->d     = d;
	l->a     = a;
	l->alpha = alpha;
	l->jvel  = 0;
	l->n = NULL;
	
	return l;
}

/**
Frees a Link from memory.

@param l The Link to be freed.
*/
void Link_free(Link* l){
	free(l);
}


/**
DualQuaternion Link Frame Transformation.

@param l The Link which DualQuaternion Frame Transformation you want.
@return A DualQuaternion representing it's transformation.
*/
DQ* Link_DQFT(Link* l){

	//Link's values
	double theta = l->theta;
	double d = l->d;
	double a = l->a;
	double alpha = l->alpha;

	//Used values
	double ct2 = cos(theta/2); double st2 = sin(theta/2);
	double ca2 = cos(alpha/2); double sa2 = sin(alpha/2);
	double d2 = d/2;           double a2 = a/2;

	//Memory Alloc	
	DQ* dq = DQ_alloc();
	Q* pp = Q_alloc();
	Q* dp = Q_alloc();
	
	//Calculation
    pp->v[0] = ct2*ca2;
	pp->v[1] = ct2*sa2;
	pp->v[2] = st2*sa2;
	pp->v[3] = st2*ca2;
	dp->v[0] = (-d2)*pp->v[3] - (a2)*pp->v[1];
	dp->v[1] = (-d2)*pp->v[2] + (a2)*pp->v[0];
	dp->v[2] =  (d2)*pp->v[1] + (a2)*pp->v[3];
	dp->v[3] =  (d2)*pp->v[0] - (a2)*pp->v[2];
	
	//Atrib
	dq->p = pp;
	dq->d = dp;
	
	return dq;
}

