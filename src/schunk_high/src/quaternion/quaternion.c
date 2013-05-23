/**
Quaternion Library.

@author Murilo Marques Marinho.
*/

#include<stdlib.h>
#include<stdio.h>
#include<math.h>

#include<schunk_high/quaternion.h>
/**
Allocates a Quaternion, its components are not initialized. Use with care.

@return a Quaternion* to the newly allocated region.
*/
Q* Q_alloc(){
	return (Q*)malloc(sizeof(Q));
}

/**
Gets the "PITCH" angle.

@param q The Unit Quaternion representing the rotation.
@return a Double with the desired value in radians.
*/
double Q_getPitchAngle(Q* q){
	double ret;

	ret = asin(2*(q->v[0]*q->v[2]-q->v[3]*q->v[1]));

	return ret;
}


/**
Allocates a Quaternion and its values.

@param a The scalar part.
@param b The part following the i unity vector.
@param c The part following the j unity vector.
@param d The part following the k unity vector.
@return A Quaternion = [a + ib + jc + kd].
*/
Q* Q_create(double a, double b, double c, double d){
	Q* q = Q_alloc();
	q->v[0] = a;	q->v[1] = b;	q->v[2] = c;	q->v[3] = d;
return q;
}

/**
Makes a copy of a Quaternion.

@param q The Quaternion you want to copy.
@return a Quaternion copy.
*/
Q* Q_copy(Q* q){
	Q* qc = Q_create(q->v[0],q->v[1],q->v[2],q->v[3]);
	return qc;
}


/**
Frees an Allocated Quaternion from memory.

@param q A Quaternion* you wish to free.
*/
void Q_free(Q* q){
	free(q);
}

/**
Prints a Quaternion.

@param q The Quaternion you want to print.
*/
void Q_print(Q* q){
	printf("\t[%f %f %f %f]\n",q->v[0],q->v[1],q->v[2],q->v[3]);
}



/**
Quaternion Addition.

Sums each element of each Quaternion.
The result is a newly allocated Quaternion with the result.
Quaternion Addition is commutative.

@param a The first Quaternion.
@param b The second Quaternion.
@param freeA TRUE if you want to free a from memory.
@param freeB TRUE if you want to free b from memory.
@return c = a+b.
*/
Q* Q_add(Q* a, Q* b, int freeA, int freeB){

	Q* c = Q_alloc();
	
	c->v[0] = a->v[0]+b->v[0];
	c->v[1] = a->v[1]+b->v[1];
	c->v[2] = a->v[2]+b->v[2];
	c->v[3] = a->v[3]+b->v[3];
	
	if(freeA){Q_free(a);}
	if(freeB){Q_free(b);}

	return c;
}

/**
Quaternion Subtraction.

Subtracts each element of each Quaternion.
The result is a newly allocated Quaternion with the result.
Quaternion Subtraction is commutative.

@param a The first Quaternion.
@param b The second Quaternion.
@param freeA TRUE if you want to free a from memory.
@param freeB TRUE if you want to free b from memory.
@return c = a-b.
*/
Q* Q_sub(Q* a, Q* b, int freeA, int freeB){

	Q* c = Q_alloc();
	
	c->v[0] = a->v[0]-b->v[0];
	c->v[1] = a->v[1]-b->v[1];
	c->v[2] = a->v[2]-b->v[2];
	c->v[3] = a->v[3]-b->v[3];
	
	if(freeA){Q_free(a);}
	if(freeB){Q_free(b);}

	return c;
}




/**
Quaternion Multiplication.

It uses the H+ operator on the first term to make the multiplication. 
Quaternion Multiplication is not commutative in general.

@param a The first Quaternion
@param b The second Quaternion
@param freeA TRUE if you want to free a from memory.
@param freeB TRUE if you want to free b from memory.
@return c = a*b
*/
Q* Q_mult(Q* a, Q* b, int freeA, int freeB){

	Q* c = Q_alloc();
	
	c->v[0] = a->v[0]*b->v[0] - a->v[1]*b->v[1] - a->v[2]*b->v[2] - a->v[3]*b->v[3];
	c->v[1] = a->v[1]*b->v[0] + a->v[0]*b->v[1] - a->v[3]*b->v[2] + a->v[2]*b->v[3];
	c->v[2] = a->v[2]*b->v[0] + a->v[3]*b->v[1] + a->v[0]*b->v[2] - a->v[1]*b->v[3];
	c->v[3] = a->v[3]*b->v[0] - a->v[2]*b->v[1] + a->v[1]*b->v[2] + a->v[0]*b->v[3];

	if(freeA){Q_free(a);}
	if(freeB){Q_free(b);}

	return c;
}

/**
Quaternion Multiplication by Double.

Multiplies each one of its terms by the desired double.

@param a The double.
@param b The Quaternion.
@param freeA TRUE if you want to free a from memory.
@param freeB TRUE if you want to free b from memory.
@return c = a*b
*/
Q* Q_multd(double a, Q*b, int freeB){

	Q* c = Q_create(b->v[0]*a,b->v[1]*a,b->v[2]*a,b->v[3]*a);
	
	if(freeB){Q_free(b);}

	return c;
}

/**
Quaternion Conjugate.

@param a The entry Quaternion.
@param freeA TRUE if you want to free a from memory.
@return a Quaternion representing the Conjugate of a, using the
operation: if a = (a1,a2,a3,a4), Q_getConj(a) = (a1,-a2,-a3,-a4).
*/
Q* Q_getConj(Q* a, int freeA){
	Q* b = Q_create(a->v[0],-(a->v[1]),-(a->v[2]),-(a->v[3]));


	if(freeA){Q_free(a);}

	return b;
}

