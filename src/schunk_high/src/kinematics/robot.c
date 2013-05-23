/**
Robot Library.

Hold's the information of a Robot (The first Link and following Links),
Robot's Forward Kinematics, and Jacobians for the Robot's Differential
Kinematics, all in the DualQuaternion space.

@author Murilo Marques Marinho.
*/

#include <stdlib.h>
#include<stdio.h>
#include<math.h>
#include<string.h>

#include<schunk_high/robot.h>
#include<schunk_high/link.h>
#include<schunk_high/gmatrix.h>
#include<schunk_high/gmatrix_plus.h>

#ifndef FALSE
	#define FALSE 0
#endif
#ifndef TRUE
	#define TRUE 1
#endif

/**

*/
DQ* Robot_getP(DQ* dq);


/**
Allocates and Initializes a Robot's variables.

@return A initialized robot with zero degrees of freedom and no Links.
*/
Robot* Robot_create(void){
	
	Robot* r = (Robot*)malloc(sizeof(Robot));
	
	r->dofs = 0;
	r->fl = NULL; 
	
	return r;
}

/**
Frees a Robot and all its Links.
*/
void Robot_free(Robot* r){
	int i = 0;

	for(i = (r->dofs - 1);i>=0;i--){
		Link_free(Robot_getLink(r,i));	
	}
	free(r);
}
/**
Adds a Link to a given robot.

@param r The Robot.
@param theta Link's DH's theta.
@param d Link's DH's d.
@param a Link's DH's a.
@param alpha Link's DH's theta.
*/
void Robot_addLink(Robot* r,double theta, double d, double a, double alpha){
	Link* i;
	Link* l;

	r->dofs++;
	
	l = Link_create(theta,d,a,alpha);
	
	if(r->fl == NULL){
		r->fl = l;
	}
	else{
		for(i = r->fl; i->n != NULL; i = i->n);	
		i->n = l;
	}

}

/**
Finds the DualQuaternion that represents the End Effector's pose using the Foward Kinematics DH Model.
The robot should have already been initialized using Robot_create() and the Links should have already been
added with Robot_addLink().

@param r The Robot which End Effector's pose is desired.
@return A DualQuaternion representing its End Effector's pose.
@see Robot_create().
@see Robot_addLink().
*/
DQ* Robot_FKM(Robot*r){

	//Iterator
	Link* i = NULL;
	
	//End Effector DualQuaternion Pose
	DQ* efp = DQ_create(1,0,0,0,0,0,0,0);

	//Goes through each link frame transformation
	for(i = r->fl; i != NULL; i = i->n){
		efp = DQ_mult(efp,Link_DQFT(i),TRUE,TRUE);
	}

	return efp;
}


/**
Gets a Link by it's index.
Trying to get a Link from a unitialized pointer leads
to an unspecified result.

@param r The Robot which Links you want to find.
@param i The index of the Link (begins on zero and ends in dofs-1).
@return The pointer to the Link or NULL if the index
is outside the Robot's Link index range or if r is
a NULL pointer.
*/
Link* Robot_getLink(Robot* r, int i){
	//Decl
	Link* l;
	int j = 0;


	if(i < 0 || i >= r->dofs){
		return NULL;
	}
	if(r == NULL){
		return NULL;
	}

	l = r->fl;
	for(j = 0; j != i; j++){
		l = l->n;
	}
	return l;
}


/**
Updates a Revolutionary Joint only thetas.

@param r The entry Robot.
@param thetas A array of doubles which size should be >= the number of links or the behavior
is unspecified and possibly dangerous.
@return 0 if the Robot is not NULL or -1 otherwise.
*/
int Robot_updateLinksThetasByArray(Robot*r, double* thetas){

	//Decl
	Link* l;
	int j = 0;

	if(r == NULL){
		return -1;
	}
	
	for(l = r->fl,j=0; l!=NULL; l=l->n,j++){
		l->theta = thetas[j];
	}
	return 0;

}


int Robot_updateLinksThetasByMatrix(Robot*r, PGMATRIX thetas){

	//Decl
	Link* l;
	int j = 0;

	if(r == NULL){
		return -1;
	}
	
	for(l = r->fl,j=0; l!=NULL; l=l->n,j++){
		l->theta = PGMATRIX_DATA(thetas,j+1,1);
	}
	return 0;

}



/**
Gets a usefull term in the Jacobian Calculation, called P in Bruno Adorno's work.

@param dq The entry quaternion.
@return The p term.
*/
DQ* Robot_getP(DQ* dq){

	DQ* ret;

	double p1;double p2;
	double p3;double p4;
	double p5;double p6;
	double p7;double p8;

	double q1 = dq->p->v[0];double q2 = dq->p->v[1];
	double q3 = dq->p->v[2];double q4 = dq->p->v[3];
	double q5 = dq->d->v[0];double q6 = dq->d->v[1];
	double q7 = dq->d->v[2];double q8 = dq->d->v[3];

	p1 = 0;
	p2 = q2*q4 + q1*q3;
    p3=q3*q4 - q1* q2;
    p4=((q4*q4)-(q3*q3)-(q2*q2)+(q1*q1))/2;
    p5=0;
    p6=q2*q8+q6*q4+q1*q7+q5*q3;
    p7=q3*q8+q7*q4-q1*q6-q5*q2;
    p8=q4*q8-q3*q7-q2*q6+q1*q5;
   

	ret = DQ_create(p1,p2,p3,p4,p5,p6,p7,p8);

return ret;
}


/**
Returns the (Analytical) DualQuaternion Jacobian.
It uses the current values in the robot's variables.

@param r The robot.
@return A (8,DOFS) matrix.
*/
PGMATRIX Robot_getAJacobian(Robot* r){

	int i = 0;
	int dofs = r->dofs;
	DQ* p;
	DQ* efp = NULL;
	DQ* dq = NULL;
	PGMATRIX AJ = NULL;

	if(dofs < 1){return NULL;}

	//End Effectors Pose.
	efp = Robot_FKM(r);
	dq = DQ_create(1,0,0,0,0,0,0,0);
	//Initialize Analytical Jacobian
	AJ = PGMATRIX_ALLOC(8,dofs);
	PGMATRIX_ZEROES(AJ);

	for(i=0;i<dofs;i++){
		
		p = Robot_getP(dq);
		dq = DQ_mult(dq,Link_DQFT(Robot_getLink(r,i)),TRUE,TRUE);
		p =  DQ_mult(p,efp,TRUE,FALSE);

		PGMATRIX_SET_COLUMN_BY_DQ(AJ,i+1,p);	
		DQ_free(p);
	}

	//Memory usage care
	DQ_free(efp);
	DQ_free(dq);

	return AJ;
}


/**
Returns the (Geometrical) DualQuaternion Jacobian.
It uses the current values in the robot's variables.
Lines refer to: (Vx, Vy, Vz, Wx, Wy, Wz)^T

@param r The robot.
@return A (6,DOFS) matrix.
*/
PGMATRIX Robot_getGJacobian(Robot* r){
	int i = 0;
	int dofs = r->dofs;
	PGMATRIX zmat;
	PGMATRIX pepimat;
	PGMATRIX zmatpepimat;
	double n=0,ex=0,ey=0,ez=0;
	Q* qpe;	Q* qpi; Q* qsub;
	DQ* efp = NULL;DQ* dq = NULL;
	PGMATRIX GJ = NULL;

	if(dofs < 1){return NULL;}

	//End Effector's Pose.
	efp = Robot_FKM(r);
	//End Effector's translation.
	qpe = DQ_getT(efp);
	
	//DQ_print(efp);
	dq = DQ_create(1,0,0,0,0,0,0,0);
	//GJ = Mat_zeros(6,dofs);
	GJ   = PGMATRIX_ALLOC(6,dofs);
	PGMATRIX_ZEROES(GJ);
	//zmat = Mat_createByArray(3,1,z)
	zmat = PGMATRIX_ALLOC(3,1);
	//pepimat = Mat_createByArray(3,1,pepi);
	pepimat = PGMATRIX_ALLOC(3,1);
	zmatpepimat = PGMATRIX_ALLOC(3,1);

	//A primeira transforma��o � a do base frame. Do DualQuaternion (1,0,0,0,0,0,0,0) e vai at� o link
	//anterior ao �ltimo.
	for(i=0;i<(dofs);i++){
		
		//Define operation
		n = dq->p->v[0];
		ex = dq->p->v[1];
		ey = dq->p->v[2];
		ez = dq->p->v[3];

		PGMATRIX_DATA(zmat,1,1) = 2*(ex*ez + n*ey);
		PGMATRIX_DATA(zmat,2,1) = 2*(ey*ez - n*ex);
		PGMATRIX_DATA(zmat,3,1) = 2*(n*n + ez*ez) - 1;
		//zmat = Mat_createByArray(3,1,z);
		///...
		///(pe-pi)
		qpi = DQ_getT(dq);
		qsub = Q_sub(qpe,qpi,FALSE,TRUE);
		PGMATRIX_DATA(pepimat,1,1) = qsub->v[1];
		PGMATRIX_DATA(pepimat,2,1) = qsub->v[2];
		PGMATRIX_DATA(pepimat,3,1) = qsub->v[3];
		//pepimat = Mat_createByArray(3,1,pepi);
		///z_(i-1)x(pe-pi)
		PGMATRIX_CROSS_COPY(zmatpepimat, zmat, pepimat);
		//zmatpepimat = Mat_crossMult3(zmat,pepimat,TRUE,TRUE);
		///Update GJ
		PGMATRIX_DATA(GJ,1,i+1) = PGMATRIX_DATA(zmatpepimat,1,1);
		PGMATRIX_DATA(GJ,2,i+1) = PGMATRIX_DATA(zmatpepimat,2,1);
		PGMATRIX_DATA(GJ,3,i+1) = PGMATRIX_DATA(zmatpepimat,3,1);
		PGMATRIX_DATA(GJ,4,i+1) = PGMATRIX_DATA(zmat,1,1);
		PGMATRIX_DATA(GJ,5,i+1) = PGMATRIX_DATA(zmat,2,1);
		PGMATRIX_DATA(GJ,6,i+1) = PGMATRIX_DATA(zmat,3,1);
		//Mat_setColumnByArray(GJ,i+1,zpepi);
		
		///Mem Free
		Q_free(qsub);
		dq = DQ_mult(dq,Link_DQFT(Robot_getLink(r,i)),TRUE,TRUE);
	}

	//Final Mem Free
	Q_free(qpe);
	DQ_free(efp);
	DQ_free(dq);
	PGMATRIX_FREE(zmat);
	PGMATRIX_FREE(pepimat);
	PGMATRIX_FREE(zmatpepimat);

return GJ;
}



/**
Returns the (Orientation) DualQuaternion Jacobian for the P angle.
It uses the current values in the robot's variables.

@param r The robot.
@return A (1,DOFS) matrix.
*/
PGMATRIX Robot_getOJacobianAngleP(Robot* r){

	DQ* efp = NULL;
	PGMATRIX AJ = NULL;
	PGMATRIX OJP = NULL;
	double doubleAux = 0;
	double q1,q2,q3,q4;
	PGMATRIX Maux1;
	PGMATRIX Maux2;

	AJ = Robot_getAJacobian(r);
	efp = Robot_FKM(r);

	q1 = efp->p->v[0];
	q2 = efp->p->v[1];
	q3 = efp->p->v[2];
	q4 = efp->p->v[3];

	Maux1 = PGMATRIX_ALLOC(1,4);
	doubleAux = 2/(sqrt(1-4*pow((q1*q3-q4*q2),2)));

	PGMATRIX_DATA(Maux1,1,1) =  q3*doubleAux;
	PGMATRIX_DATA(Maux1,1,2) = -q4*doubleAux;
	PGMATRIX_DATA(Maux1,1,3) =  q1*doubleAux;
	PGMATRIX_DATA(Maux1,1,4) = -q2*doubleAux;

	Maux2 = PGMATRIX_ALLOC(4,r->dofs);

	PGMATRIX_COPY_ROW(Maux2, 1, AJ, 1);
	PGMATRIX_COPY_ROW(Maux2, 2, AJ, 2);
	PGMATRIX_COPY_ROW(Maux2, 3, AJ, 3);
	PGMATRIX_COPY_ROW(Maux2, 4, AJ, 4);

	OJP = PGMATRIX_ALLOC(1,r->dofs);
	PGMATRIX_MULTIPLY_COPY(OJP, Maux1, Maux2);

	PGMATRIX_FREE(Maux1);
	
	PGMATRIX_FREE(Maux2);
	
	PGMATRIX_FREE(AJ);
	
	DQ_free(efp);

	return OJP;
}

//PGMATRIX Robot_getSecondaryObjectives(Robot* r, char* type, PGMATRIX Thetas, PGMATRIX DThetas, PGMATRIX SJ){
// Siciliano 3.55
//	PGMATRIX qo = NULL;
//
//	PGMATRIX pAux1 = NULL;
//	PGMATRIX SJt   = NULL;
//
//	double det = 0;
//	double det_f = 0;
//
//
//	if(!strcmp("MM",type)){
//	
//		SJt = PGMATRIX_ALLOC(SJ->Nc,SJ->Nr);
//		PGMATRIX_TRANSPOSE_COPY(SJt,SJ);
//
//		pAux1 = PGMATRIX_MULTIPLY_COPY_FREE();
//
//		det = 
//
//
//
//	}
//
//
//	return qo;
//}
