#include<schunk_high/gmatrix.h>
#include<schunk_high/gmatrix_linalg.h>
#include<schunk_high/gmatrix_plus.h>
#include<schunk_high/dualquaternion.h>

#include<stdio.h>
#include<stdlib.h>
#include<math.h>

#ifndef TRUE
	#define TRUE 1
#endif
#ifndef FALSE
	#define FALSE 0
#endif

void PGMATRIX_SET_COLUMN_BY_DQ(PGMATRIX Pmat, int col, DQ* dq ){

	GMATRIX_ASSERT("PGMATRIX_SET_COLUMN_BY_DQ",(col > Pmat->Nc)&&(Pmat->Nr != 8));

	PGMATRIX_DATA(Pmat,1,col) = dq->p->v[0];
	PGMATRIX_DATA(Pmat,2,col) = dq->p->v[1];
	PGMATRIX_DATA(Pmat,3,col) = dq->p->v[2];
	PGMATRIX_DATA(Pmat,4,col) = dq->p->v[3];
	PGMATRIX_DATA(Pmat,5,col) = dq->d->v[0];
	PGMATRIX_DATA(Pmat,6,col) = dq->d->v[1];
	PGMATRIX_DATA(Pmat,7,col) = dq->d->v[2];
	PGMATRIX_DATA(Pmat,8,col) = dq->d->v[3];

}

PGMATRIX PGMATRIX_CREATE_DIAG_FROM_ARRAY(int Dim, GMATRIX_FLOAT* Array){
	
	int i = 0, j = 0;

	PGMATRIX Pmat = PGMATRIX_ALLOC(Dim,Dim);

	for(i = 0;i<Dim;i++){
		for(j=0;j<Dim;j++){
			if( i == j){
				PGMATRIX_DATA(Pmat,i+1,j+1) = Array[i];
			}
			else{
				PGMATRIX_DATA(Pmat,i+1,j+1) = 0;
			}
		}
	}

	return Pmat;
}

PGMATRIX PGMATRIX_CREATE_FROM_ARRAY(int Nr, int Nc, GMATRIX_FLOAT* Array){
	
	int i = 0, j = 0;

	PGMATRIX Pmat = PGMATRIX_ALLOC(Nr,Nc);

	for(i = 0;i<Nr;i++){
		for(j=0;j<Nc;j++){
				PGMATRIX_DATA(Pmat,i+1,j+1) = Array[i+j];
		}
	}

	return Pmat;
}

PGMATRIX PGMATRIX_CREATE_ZEROES(int Nr, int Nc){

	PGMATRIX pMat = PGMATRIX_ALLOC(Nr,Nc);
	PGMATRIX_ZEROES(pMat);

	return pMat;
}

PGMATRIX PGMATRIX_SUBTRACT_COPY_FREE(PGMATRIX pMatA, PGMATRIX pMatB, int freeA, int freeB){
	
	PGMATRIX pMatRes = PGMATRIX_CREATE_ZEROES(pMatA->Nr,pMatA->Nc);

	PGMATRIX_SUBTRACT_COPY(pMatRes, pMatA, pMatB);

	if(freeA){PGMATRIX_FREE(pMatA);}
	if(freeB){PGMATRIX_FREE(pMatB);}

	return pMatRes;
}

PGMATRIX PGMATRIX_ADD_COPY_FREE(PGMATRIX pMatA, PGMATRIX pMatB, int freeA, int freeB){
		
	PGMATRIX pMatRes = PGMATRIX_CREATE_ZEROES(pMatA->Nr,pMatA->Nc);

	PGMATRIX_ADD_COPY(pMatRes, pMatA, pMatB);

	if(freeA){PGMATRIX_FREE(pMatA);}
	if(freeB){PGMATRIX_FREE(pMatB);}

	return pMatRes;
}

PGMATRIX PGMATRIX_MULTIPLY_COPY_FREE(PGMATRIX pMatA, PGMATRIX pMatB, int freeA, int freeB){
		
	PGMATRIX pMatRes = PGMATRIX_CREATE_ZEROES(pMatA->Nr,pMatB->Nc);

	PGMATRIX_MULTIPLY_COPY(pMatRes, pMatA, pMatB);

	if(freeA){PGMATRIX_FREE(pMatA);}
	if(freeB){PGMATRIX_FREE(pMatB);}

	return pMatRes;
}

PGMATRIX PGMATRIX_MULTIPLY_CONST_COPY_FREE(PGMATRIX pMat, GMATRIX_FLOAT a, int freeMat){

	PGMATRIX pMatRes = PGMATRIX_CREATE_ZEROES(pMat->Nr,pMat->Nc);

	PGMATRIX_COPY(pMatRes,pMat);
	PGMATRIX_MULTIPLY_CONST(pMatRes, a );

	if(freeMat){PGMATRIX_FREE(pMat);}

	return pMatRes;
}

PGMATRIX PGMATRIX_PSEUDOINVERSE_PLUS(PGMATRIX pA)
{
	int n;
	GMATRIX_FLOAT tol, norm;
	PGMATRIX pU;
	PGMATRIX pS;
	PGMATRIX pV;
	PGMATRIX pApinv;
	PGMATRIX pMatDummy;

	pApinv    = PGMATRIX_CREATE_ZEROES(pA->Nr,pA->Nc);
	pMatDummy = PGMATRIX_CREATE_ZEROES(pA->Nr,pA->Nc);

	GMATRIX_ASSERT("PGMATRIX_PSEUDOINVERSE",pA->Nr <= 0);
	GMATRIX_ASSERT("PGMATRIX_PSEUDOINVERSE",pA->Nc <= 0);
	GMATRIX_ASSERT("PGMATRIX_PSEUDOINVERSE",pMatDummy->MaxSize < pA->Nr*pA->Nc);

	pU = PGMATRIX_ALLOC(pA->Nr,pA->Nc);
	pS = PGMATRIX_ALLOC(pA->Nc,pA->Nc);
	pV = PGMATRIX_ALLOC(pA->Nc,pA->Nc);

	// Compute SVD:
	PGMATRIX_SVD(pU,pS,pV,pA,FALSE);
	// Compute norm:
	norm = (GMATRIX_FLOAT)(fabs(PGMATRIX_DATA(pS,1,1)));
	for(n=2;n<=pA->Nc;++n){
		norm = GMATRIXMACRO_MAX(norm,(GMATRIX_FLOAT)(fabs(PGMATRIX_DATA(pS,n,n))));
	}
	// Compute tolerance:
	tol = (GMATRIXMACRO_MAX(pA->Nr,pA->Nc)) * norm * (GMATRIX_FLOAT)(1e-12);

	// Compute inv(S):
	for(n=1;n<=pA->Nc;++n){
		if (fabs(PGMATRIX_DATA(pS,n,n))<=tol){
			PGMATRIX_DATA(pS,n,n) = 0.0;
		}
		else{
			PGMATRIX_DATA(pS,n,n) = (GMATRIX_FLOAT)(1.0) / PGMATRIX_DATA(pS,n,n);
		}
	}
	// Compute pinv:
	PGMATRIX_MULTIPLY_COPY_EXTENDED(pMatDummy,pS,FALSE,pU,TRUE);
	PGMATRIX_MULTIPLY_COPY_EXTENDED(pApinv,pV,FALSE,pMatDummy,FALSE);

	PGMATRIX_FREE(pU);
	PGMATRIX_FREE(pS);
	PGMATRIX_FREE(pV);
	PGMATRIX_FREE(pMatDummy);

	return pApinv;
}

/**
J = J^T(J.J^T + k^2.I)−1
Siciliano EQ. 3.59
*/
PGMATRIX PGMATRIX_DAMPED_LEASTSQUARES_PSEUDOINVERSE(PGMATRIX pMat, double k){


	PGMATRIX pApinv  = NULL;
	PGMATRIX pAux1   = NULL;
	PGMATRIX kI      = NULL;
	PGMATRIX pT      = NULL;
	PGMATRIX pMatRes = NULL;
	int i=0,j=0;
	double ksqr      = 0;

	kI = PGMATRIX_ALLOC(pMat->Nr,pMat->Nr);
	pT = PGMATRIX_ALLOC(pMat->Nc,pMat->Nr);

	ksqr = ((k)*(k));

	for(i=0;i<pMat->Nr;i++){

		for(j=0;j<pMat->Nr;j++){
			if(i==j){
				PGMATRIX_DATA(kI,i+1,j+1) = ksqr;
			}
			else{
				PGMATRIX_DATA(kI,i+1,j+1) = 0;
			}
		}
	}

	PGMATRIX_TRANSPOSE_COPY(pT,pMat);

	pAux1 = PGMATRIX_MULTIPLY_COPY_FREE(pMat,pT,FALSE,FALSE);
	
	pAux1 = PGMATRIX_ADD_COPY_FREE(pAux1,kI,TRUE,TRUE);

	pApinv = PGMATRIX_PSEUDOINVERSE_PLUS(pAux1);
	PGMATRIX_FREE(pAux1);

	pMatRes = PGMATRIX_MULTIPLY_COPY_FREE(pT,pApinv,TRUE,TRUE);

	return pMatRes;
}


/**
J^cross = J^T(J*J^T)^-1
Siciliano EQ. 3.52
*/
PGMATRIX PGMATRIX_RIGHT_PSEUDOINVERSE(PGMATRIX pMat){

	PGMATRIX pApinv  = NULL;
	PGMATRIX pAux1   = NULL;
	PGMATRIX pT      = NULL;
	PGMATRIX pMatRes = NULL;

	pT = PGMATRIX_ALLOC(pMat->Nc,pMat->Nr);

	PGMATRIX_TRANSPOSE_COPY(pT,pMat);

	pAux1 = PGMATRIX_MULTIPLY_COPY_FREE(pMat,pT,FALSE,FALSE);
	
	pApinv = PGMATRIX_ALLOC(pAux1->Nr,pAux1->Nc);
	PGMATRIX_INVERSE_COPY(pApinv,pAux1);

	PGMATRIX_FREE(pAux1);

	pMatRes = PGMATRIX_MULTIPLY_COPY_FREE(pT,pApinv,TRUE,TRUE);

	return pMatRes;

}


/**
(In - J^cross*J)
Siciliano EQ. 3.54
*/
PGMATRIX PGMATRIX_RIGHT_PSEUDOINVERSE_NULLSPACE_PROJECTOR(PGMATRIX pMat){


	PGMATRIX pApinv  = NULL;
	PGMATRIX pAux1   = NULL;
	PGMATRIX In      = NULL;
	PGMATRIX pMatRes = NULL;
	int i=0,j=0;

	In = PGMATRIX_ALLOC(pMat->Nc,pMat->Nc);

	for(i=0;i<pMat->Nc;i++){

		for(j=0;j<pMat->Nc;j++){
			if(i==j){
				PGMATRIX_DATA(In,i+1,j+1) = 1;
			}
			else{
				PGMATRIX_DATA(In,i+1,j+1) = 0;
			}
		}
	}

	pApinv = PGMATRIX_RIGHT_PSEUDOINVERSE(pMat);

	pAux1 = PGMATRIX_MULTIPLY_COPY_FREE(pApinv,pMat,TRUE,FALSE);
	
	pMatRes = PGMATRIX_SUBTRACT_COPY_FREE(In,pAux1,TRUE,TRUE);

	return pMatRes;
}
