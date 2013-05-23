/**********************************************************************************************
***********************************************************************************************
***  File:			GMatrix_linalg.c
***	 Author:		Geovany Araujo Borges
***	 Contents:		Implementation of Matrix type and related functions for use in C and C++.
***					Linear algebra functions
***********************************************************************************************
***********************************************************************************************
	This file is part of gMatrix.

    gMatrix is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License (LGPL) as published by
    the Free Software Foundation, either version 3 of the License, or any later version.

    gMatrix is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with gMatrix.  If not, see <http://www.gnu.org/licenses/>.

	Copyright 2010 Geovany Araujo Borges
**********************************************************************************************/

//#include "stdafx.h" // MSVC6.0 may require this.
#include<schunk_high/gmatrix.h>
#include<schunk_high/gmatrix_linalg.h>

/**********************************************************************************************
***** GMatrix: Includes.
**********************************************************************************************/
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

/**********************************************************************************************
***** GMatrix: LU Decomposition
**********************************************************************************************/
#if GMATRIX_DEFINE_LUDCMP
GMATRIX_FLOAT PGMATRIX_LUDCMP(PGMATRIX pMatLU, PGMATRIX pMat)
{
	GMATRIX_FLOAT  big,dum,sum,temp;
	GMATRIX_FLOAT	d;
	GMATRIX_FLOAT  *vv;
	int i,imax=0,j,k;

	GMATRIX_ASSERT("GMATRIX_LUDCMP",pMat->Nr != pMat->Nc);	

	PGMATRIX_COPY(pMatLU, pMat); 

	vv = (GMATRIX_FLOAT*) malloc((pMat->Nr+1)*sizeof(GMATRIX_FLOAT));
	
	d=1.0;	
	for (i=1;i<=pMatLU->Nr;i++)  {
		big=0.0;	
		for (j=1;j<=pMatLU->Nr;j++)
			if ((temp=(GMATRIX_FLOAT)(fabs(PGMATRIX_DATA(pMatLU,i,j)))) > big) big=temp;
		if (big == 0.0) GMATRIX_ASSERT("PGMATRIX_LUDCMP: Singular matrix",1);
		vv[i]=(GMATRIX_FLOAT)(1.0)/big;
	} 
	for (j=1;j<=pMatLU->Nr;j++)  {
		for (i=1;i<j;i++) {
			sum=PGMATRIX_DATA(pMatLU,i,j);
			for (k=1;k<i;k++) sum -= PGMATRIX_DATA(pMatLU,i,k)*PGMATRIX_DATA(pMatLU,k,j);
			PGMATRIX_DATA(pMatLU,i,j)=sum;
		}
		big=0.0;
		for (i=j;i<=pMatLU->Nr;i++) {
			sum=PGMATRIX_DATA(pMatLU,i,j);
			for (k=1;k<j;k++)
				sum -= PGMATRIX_DATA(pMatLU,i,k)*PGMATRIX_DATA(pMatLU,k,j);
			PGMATRIX_DATA(pMatLU,i,j)=sum;
			if ( (dum=vv[i]*(GMATRIX_FLOAT)(fabs(sum))) >= big) {
				big=dum;
				imax=i;
			}
		}
		if (j != imax) {
			for (k=1;k<=pMatLU->Nr;k++) {
				dum=PGMATRIX_DATA(pMatLU,imax,k);
				PGMATRIX_DATA(pMatLU,imax,k)=PGMATRIX_DATA(pMatLU,j,k);
				PGMATRIX_DATA(pMatLU,j,k)=dum;
			}
			d = -(d);
			vv[imax]=vv[j];
		}
		if (PGMATRIX_DATA(pMatLU,j,j) == 0.0) PGMATRIX_DATA(pMatLU,j,j)=(GMATRIX_FLOAT)(1.0e-20);
		if (j != pMatLU->Nr) {
			dum=(GMATRIX_FLOAT)(1.0)/(PGMATRIX_DATA(pMatLU,j,j));
			for (i=j+1;i<=pMatLU->Nr;i++) PGMATRIX_DATA(pMatLU,i,j) *= dum;
		}
	}

	free(vv);

	return d;

}
#endif

/**********************************************************************************************
***** GMatrix: Gauss-Jordan Elimination
***** Abstract: Given matrices A and B from A*X=B, computes inv(A) and X. 
*****			It is applied only to determined systems, i.e. A is a square matrix.
***** Note: if the pointer to the X or B matrices is NULL, it performs only inv(A).
**********************************************************************************************/

#if GMATRIX_DEFINE_GAUSSJORDAN
void PGMATRIX_GAUSSJORDAN(PGMATRIX pAInverse, PGMATRIX pX, PGMATRIX pA, PGMATRIX pB)
{
	int i,icol=0,irow=0,j,k,l,ll;
	GMATRIX_FLOAT big,dum,pivinv,temp;
	int *indxc,*indxr,*ipiv;
	int flaginverseonly = 1;

	GMATRIX_ASSERT("PGMATRIX_GAUSSJORDAN",pA->Nr != pA->Nc);
	if ((pX!=NULL) && (pB!=NULL)){
		flaginverseonly = 0;
		GMATRIX_ASSERT("PGMATRIX_GAUSSJORDAN",pA->Nr != pB->Nr);
	}

	if (pAInverse!=pA){
		PGMATRIX_COPY(pAInverse, pA);
	}
	if (flaginverseonly==0){
		PGMATRIX_COPY(pX, pB);
	}

	indxc = (int*) malloc((pA->Nr+1)*sizeof(int));
	indxr = (int*) malloc((pA->Nr+1)*sizeof(int));
	ipiv  = (int*) malloc((pA->Nr+1)*sizeof(int));

	for (j=1;j<=pA->Nr;j++) ipiv[j]=0;
	for (i=1;i<=pA->Nr;i++) {
		big=0.0;
		for (j=1;j<=pA->Nr;j++)
			if (ipiv[j] != 1)
				for (k=1;k<=pA->Nr;k++) {
					if (ipiv[k] == 0) {
						if (fabs(PGMATRIX_DATA(pAInverse,j,k)) >= big) {
							big=(GMATRIX_FLOAT)(fabs(PGMATRIX_DATA(pAInverse,j,k)));
							irow=j;
							icol=k;
						}
					} else if (ipiv[k] > 1) GMATRIX_ASSERT("PGMATRIX_GAUSSJORDAN: Singular matrix (step 1)",1);
				}
		++(ipiv[icol]);
		if (irow != icol) {
			for (l=1;l<=pA->Nr;l++) GMATRIXMACRO_SWAP(PGMATRIX_DATA(pAInverse,irow,l),PGMATRIX_DATA(pAInverse,icol,l));
			if (flaginverseonly==0){
				for (l=1;l<=pB->Nc;l++) GMATRIXMACRO_SWAP(PGMATRIX_DATA(pX,irow,l),PGMATRIX_DATA(pX,icol,l));
			}
		}
		indxr[i]=irow;
		indxc[i]=icol;
		if (PGMATRIX_DATA(pAInverse,icol,icol) == 0.0){
			GMATRIX_ASSERT("PGMATRIX_GAUSSJORDAN: Singular matrix (step 2)",1);
		}
		pivinv=(GMATRIX_FLOAT)(1.0)/PGMATRIX_DATA(pAInverse,icol,icol);
		PGMATRIX_DATA(pAInverse,icol,icol)=1.0;
		for (l=1;l<=pA->Nr;l++) PGMATRIX_DATA(pAInverse,icol,l) *= pivinv;
		if (flaginverseonly==0){
			for (l=1;l<=pB->Nc;l++) PGMATRIX_DATA(pX,icol,l) *= pivinv;
		}
		for (ll=1;ll<=pA->Nr;ll++){
			if (ll != icol) {
				dum=PGMATRIX_DATA(pAInverse,ll,icol);
				PGMATRIX_DATA(pAInverse,ll,icol)=0.0;
				for (l=1;l<=pA->Nr;l++) PGMATRIX_DATA(pAInverse,ll,l) -= PGMATRIX_DATA(pAInverse,icol,l)*dum;
				if (flaginverseonly==0){
					for (l=1;l<=pB->Nc;l++) PGMATRIX_DATA(pX,ll,l) -= PGMATRIX_DATA(pX,icol,l)*dum;
				}
			}
		}
	}
	
	for (l=pA->Nr;l>=1;l--) {
		if (indxr[l] != indxc[l]){
			for (k=1;k<=pA->Nr;k++){
				GMATRIXMACRO_SWAP(PGMATRIX_DATA(pAInverse,k,indxr[l]),PGMATRIX_DATA(pAInverse,k,indxc[l]));
			}
		}
	}

	free(ipiv);
	free(indxr);
	free(indxc);
}
#endif

/**********************************************************************************************
***** GMatrix: Cholesky decomposition
***** Abstract: Given matrice A symmetric and positive definite, computes a lower diagonal
*****			matrix L such that L*(L)' = A. 
**********************************************************************************************/
#if GMATRIX_DEFINE_CHOLESKY
void PGMATRIX_CHOLESKY(PGMATRIX pL, PGMATRIX pA)
{
	int i,j,k;
	GMATRIX_FLOAT sum,*p;

	GMATRIX_ASSERT("PGMATRIX_CHOLESKY",pA->Nr != pA->Nc);
	PGMATRIX_COPY(pL,pA);	
	p = (GMATRIX_FLOAT*) malloc((pA->Nr+1)*sizeof(GMATRIX_FLOAT));

	for (i=1;i<=pL->Nr;i++) {
		for (j=i;j<=pL->Nr;j++) {
			for (sum=PGMATRIX_DATA(pL,i,j),k=i-1;k>=1;k--){
				sum -= PGMATRIX_DATA(pL,i,k)*PGMATRIX_DATA(pL,j,k);
			}
			if (i == j) {
				if (sum == 0.0){ // For singular matrices (incomplet Cholesky form);
					sum = (GMATRIX_FLOAT)(1e-50);
				}
				if (sum <= 0.0){
//				if (sum <= 0.0){
					free(p);
					GMATRIX_ASSERT("PGMATRIX_CHOLESKY: Failed (Singular or non-positive definite matrix",1);
					return;
				}
				p[i]=(GMATRIX_FLOAT)(sqrt(sum));
			} else PGMATRIX_DATA(pL,j,i)=sum/p[i];
		}
	}
	for (i=1;i<=pL->Nr;i++) {
		for (j=i;j<=pL->Nc;j++) {
			if (i == j) {
				PGMATRIX_DATA(pL,i,j)=p[i];
			}
			else{
				PGMATRIX_DATA(pL,i,j)=0.0;
			}
		}
	}
	free(p);
}
#endif

/**********************************************************************************************
***** GMatrix: Determinant
**********************************************************************************************/
#if GMATRIX_DEFINE_DETERMINANT2
GMATRIX_FLOAT PGMATRIX_DETERMINANT2(PGMATRIX pMat)
{
	GMATRIX_FLOAT Determinant = 0.0;

	GMATRIX_ASSERT("PGMATRIX_DETERMINANT",pMat->Nr != pMat->Nc);	
	GMATRIX_ASSERT("PGMATRIX_DETERMINANT",pMat->Nr == 0);	
	GMATRIX_ASSERT("PGMATRIX_DETERMINANT",pMat->Nr != 2);	
	
	Determinant = PGMATRIX_DATA(pMat,2,2)*PGMATRIX_DATA(pMat,1,1)-PGMATRIX_DATA(pMat,1,2)*PGMATRIX_DATA(pMat,2,1);
	
	return Determinant;
}
#endif

#if GMATRIX_DEFINE_DETERMINANT3
GMATRIX_FLOAT PGMATRIX_DETERMINANT3(PGMATRIX pMat)
{
	GMATRIX_FLOAT Determinant = 0.0;

	GMATRIX_ASSERT("PGMATRIX_DETERMINANT",pMat->Nr != pMat->Nc);	
	GMATRIX_ASSERT("PGMATRIX_DETERMINANT",pMat->Nr == 0);	
	GMATRIX_ASSERT("PGMATRIX_DETERMINANT",pMat->Nr != 3);	

	Determinant = (PGMATRIX_DATA(pMat,1,1)*PGMATRIX_DATA(pMat,2,2)*PGMATRIX_DATA(pMat,3,3)-PGMATRIX_DATA(pMat,1,1)*PGMATRIX_DATA(pMat,2,3)*PGMATRIX_DATA(pMat,3,2)-PGMATRIX_DATA(pMat,2,1)*PGMATRIX_DATA(pMat,1,2)*PGMATRIX_DATA(pMat,3,3)+PGMATRIX_DATA(pMat,2,1)*PGMATRIX_DATA(pMat,1,3)*PGMATRIX_DATA(pMat,3,2)+PGMATRIX_DATA(pMat,3,1)*PGMATRIX_DATA(pMat,1,2)*PGMATRIX_DATA(pMat,2,3)-PGMATRIX_DATA(pMat,3,1)*PGMATRIX_DATA(pMat,1,3)*PGMATRIX_DATA(pMat,2,2));

	return Determinant;
}
#endif

#if GMATRIX_DEFINE_DETERMINANT
GMATRIX_FLOAT PGMATRIX_DETERMINANT(PGMATRIX pMat, PGMATRIX pMatDummy)
{
	GMATRIX_FLOAT Determinant = 0.0;
	int j;

	GMATRIX_ASSERT("PGMATRIX_DETERMINANT",pMat->Nr != pMat->Nc);	
	GMATRIX_ASSERT("PGMATRIX_DETERMINANT",pMat->Nr == 0);	

	if (pMat->Nc == 2){	
		return PGMATRIX_DETERMINANT2(pMat);
	} 
	if (pMat->Nc == 3){	
		return PGMATRIX_DETERMINANT3(pMat);
	} 
	else{	
		Determinant = PGMATRIX_LUDCMP(pMatDummy,pMat); // MatDummy is the LU decomposition of Mat  
		
		for (j=1; j<=pMatDummy->Nr; ++j){ 
			Determinant *= PGMATRIX_DATA(pMatDummy,j,j); 
		} 

		return Determinant;
	}
}
#endif

/**********************************************************************************************
***** GMatrix: Eigenvalues
**********************************************************************************************/
/*#define GMATRIX_EIGENVALUES2(Eigenvalue1, Eigenvalue1, Mat) PGMATRIX_EIGENVALUES2(&Eigenvalue1,&Eigenvalue2, &Mat)

void PGMATRIX_EIGENVALUES2(GMATRIX_FLOAT* pEigenvalue1, GMATRIX_FLOAT* pEigenvalue2, PGMATRIX pMat) 
{
	GMATRIX_ASSERT("PGMATRIX_EIGENVALUES2",pMat->Nr != 2);
	GMATRIX_ASSERT("PGMATRIX_EIGENVALUES2",pMat->Nr != pMat->Nc);	

	*pEigenvalue1 = 0.5*(M[0][0] + M[1][1] + sqrt( (M[0][0] - M[1][1])*(M[0][0] - M[1][1]) + 4*M[1][0]*M[0][1] ));
	*pEigenvalue2 = 0.5*(M[0][0] + M[1][1] - sqrt( (M[0][0] - M[1][1])*(M[0][0] - M[1][1]) + 4*M[1][0]*M[0][1] ));

}*/

/**********************************************************************************************
***** GMatrix: Inverse
**********************************************************************************************/
#if GMATRIX_DEFINE_INVERSE2_COPY
void PGMATRIX_INVERSE2_COPY(PGMATRIX pMatInverse, PGMATRIX pMat) 
{
	GMATRIX_FLOAT Det;

	GMATRIX_ASSERT("PGMATRIX_INVERSE2_COPY",pMat->Nr != pMat->Nc);	
	GMATRIX_ASSERT("PGMATRIX_INVERSE2_COPY",pMat->Nr == 0);	
	GMATRIX_ASSERT("PGMATRIX_INVERSE2_COPY",pMat->Nr != 2);
	PGMATRIX_SETSIZE(pMatInverse,pMat->Nr, pMat->Nc);
	Det = PGMATRIX_DETERMINANT2(pMat);

	if ( fabs(Det) >= 1e-12 ){ 
		PGMATRIX_DATA(pMatInverse,1,1) =  PGMATRIX_DATA(pMat,2,2)/Det; 
		PGMATRIX_DATA(pMatInverse,1,2) = -PGMATRIX_DATA(pMat,1,2)/Det; 
		PGMATRIX_DATA(pMatInverse,2,1) = -PGMATRIX_DATA(pMat,2,1)/Det; 
		PGMATRIX_DATA(pMatInverse,2,2) =  PGMATRIX_DATA(pMat,1,1)/Det; 
	}	
	else{ 
		PGMATRIX_GAUSSJORDAN(pMatInverse, NULL, pMat, NULL);
	}
}
#endif
      
#if GMATRIX_DEFINE_INVERSE3_COPY
void PGMATRIX_INVERSE3_COPY(PGMATRIX pMatInverse, PGMATRIX pMat) 
{
	GMATRIX_FLOAT Det;

	GMATRIX_ASSERT("PGMATRIX_INVERSE3_COPY",pMat->Nr != pMat->Nc);	
	GMATRIX_ASSERT("PGMATRIX_INVERSE3_COPY",pMat->Nr == 0);	
	GMATRIX_ASSERT("PGMATRIX_INVERSE3_COPY",pMat->Nr != 3);	
	PGMATRIX_SETSIZE(pMatInverse,pMat->Nr, pMat->Nc);
	Det = PGMATRIX_DETERMINANT3(pMat);

	if ( fabs(Det) >= 1e-12 ){ 
    	PGMATRIX_DATA(pMatInverse,1,1) =  (PGMATRIX_DATA(pMat,2,2)*PGMATRIX_DATA(pMat,3,3)-PGMATRIX_DATA(pMat,2,3)*PGMATRIX_DATA(pMat,3,2))/Det; 
	    PGMATRIX_DATA(pMatInverse,1,2) = -(PGMATRIX_DATA(pMat,1,2)*PGMATRIX_DATA(pMat,3,3)-PGMATRIX_DATA(pMat,1,3)*PGMATRIX_DATA(pMat,3,2))/Det; 
    	PGMATRIX_DATA(pMatInverse,1,3) =  (PGMATRIX_DATA(pMat,1,2)*PGMATRIX_DATA(pMat,2,3)-PGMATRIX_DATA(pMat,1,3)*PGMATRIX_DATA(pMat,2,2))/Det; 
	    PGMATRIX_DATA(pMatInverse,2,1) = -(PGMATRIX_DATA(pMat,2,1)*PGMATRIX_DATA(pMat,3,3)-PGMATRIX_DATA(pMat,2,3)*PGMATRIX_DATA(pMat,3,1))/Det; 
    	PGMATRIX_DATA(pMatInverse,2,2) =  (PGMATRIX_DATA(pMat,1,1)*PGMATRIX_DATA(pMat,3,3)-PGMATRIX_DATA(pMat,1,3)*PGMATRIX_DATA(pMat,3,1))/Det; 
	    PGMATRIX_DATA(pMatInverse,2,3) = -(PGMATRIX_DATA(pMat,1,1)*PGMATRIX_DATA(pMat,2,3)-PGMATRIX_DATA(pMat,1,3)*PGMATRIX_DATA(pMat,2,1))/Det; 
    	PGMATRIX_DATA(pMatInverse,3,1) =  (PGMATRIX_DATA(pMat,2,1)*PGMATRIX_DATA(pMat,3,2)-PGMATRIX_DATA(pMat,2,2)*PGMATRIX_DATA(pMat,3,1))/Det; 
	    PGMATRIX_DATA(pMatInverse,3,2) = -(PGMATRIX_DATA(pMat,1,1)*PGMATRIX_DATA(pMat,3,2)-PGMATRIX_DATA(pMat,1,2)*PGMATRIX_DATA(pMat,3,1))/Det; 
    	PGMATRIX_DATA(pMatInverse,3,3) =  (PGMATRIX_DATA(pMat,1,1)*PGMATRIX_DATA(pMat,2,2)-PGMATRIX_DATA(pMat,1,2)*PGMATRIX_DATA(pMat,2,1))/Det; 
	} 
	else{ 
		PGMATRIX_GAUSSJORDAN(pMatInverse, NULL, pMat, NULL);
	}
}
#endif

#if GMATRIX_DEFINE_INVERSE_COPY
void PGMATRIX_INVERSE_COPY(PGMATRIX pMatInverse, PGMATRIX pMat) 
{
	GMATRIX_ASSERT("PGMATRIX_INVERSE_COPY",pMat->Nr != pMat->Nc);
	GMATRIX_ASSERT("PGMATRIX_INVERSE_COPY",pMat->Nr == 0);	

	if (pMat->Nc == 1){	
		GMATRIX_ASSERT("PGMATRIX_INVERSE_COPY",pMat->Nr != pMat->Nc);	
		if (fabs(PGMATRIX_DATA(pMat,1,1)) >= 1e-16){
			PGMATRIX_SETSIZE(pMatInverse,pMat->Nr, pMat->Nc);
			PGMATRIX_DATA(pMatInverse,1,1) = (GMATRIX_FLOAT)(1.0)/PGMATRIX_DATA(pMat,1,1); 
		}
		else{
			GMATRIX_ASSERT("PGMATRIX_INVERSE_COPY: Singular matrix",1);	
		}
	}
	if (pMat->Nc == 2){	
		PGMATRIX_INVERSE2_COPY(pMatInverse,pMat);
	} 
	if (pMat->Nc == 3){	
		PGMATRIX_INVERSE3_COPY(pMatInverse,pMat);
	} 
	if (pMat->Nc > 3){	
		PGMATRIX_GAUSSJORDAN(pMatInverse, NULL, pMat, NULL);
	}
}
#endif

#if GMATRIX_DEFINE_INVERSE
void PGMATRIX_INVERSE(PGMATRIX pMat) 
{
	GMATRIX_ASSERT("PGMATRIX_INVERSE_COPY",pMat->Nr != pMat->Nc);
	GMATRIX_ASSERT("PGMATRIX_INVERSE_COPY",pMat->Nr == 0);	
	PGMATRIX_GAUSSJORDAN(pMat, NULL, pMat, NULL);
}
#endif

/**********************************************************************************************
***** GMatrix: Singular Valor Decomposition: Mat = U*S*Transpose(V)
**********************************************************************************************/
#if GMATRIX_DEFINE_SVD
GMATRIX_FLOAT dpythag(GMATRIX_FLOAT a, GMATRIX_FLOAT b)
{
	GMATRIX_FLOAT absa,absb;
	absa=(GMATRIX_FLOAT)(fabs(a));
	absb=(GMATRIX_FLOAT)(fabs(b));
	if (absa > absb) return absa*((GMATRIX_FLOAT)(sqrt(1.0+GMATRIXMACRO_SQR(absb/absa))));
	else return (absb == 0.0 ? (GMATRIX_FLOAT)(0.0) : absb*((GMATRIX_FLOAT)(sqrt(1.0+GMATRIXMACRO_SQR(absa/absb)))));
}

void PGMATRIX_SVD(PGMATRIX pU,PGMATRIX pS,PGMATRIX pV,PGMATRIX pMat, unsigned char FlagSorted)
{
	int ir;
	GMATRIX_FLOAT cS;
	int flag,i,its,j,jj,k,l,nm;
	GMATRIX_FLOAT anorm,c,f,g,h,s,scale,x,y,z;
	PGMATRIX pRV1,pcU,pcV;

	GMATRIX_ASSERT("PGMATRIX_SVD",pMat->Nr <= 0);
	GMATRIX_ASSERT("PGMATRIX_SVD",pMat->Nc <= 0);

	PGMATRIX_SETSIZE(pU,pMat->Nr,pMat->Nc);
	PGMATRIX_SETSIZE(pS,pMat->Nc,pMat->Nc);
	PGMATRIX_SETSIZE(pV,pMat->Nc,pMat->Nc);

	pRV1 = PGMATRIX_ALLOC(pMat->Nc,1);
	PGMATRIX_COPY(pU,pMat);
	PGMATRIX_ZEROES(pS);

	g=scale=anorm=0.0;
	for (i=1;i<=pMat->Nc;i++) {
		l=i+1;
		PGMATRIX_DATA(pRV1,i,1)=scale*g;
		g=s=scale=0.0;
		if (i <= pMat->Nr) {
			for (k=i;k<=pMat->Nr;k++) scale += (GMATRIX_FLOAT)(fabs(PGMATRIX_DATA(pU,k,i)));
			if (scale) {
				for (k=i;k<=pMat->Nr;k++) {
					PGMATRIX_DATA(pU,k,i) /= scale;
					s += PGMATRIX_DATA(pU,k,i)*PGMATRIX_DATA(pU,k,i);
				}
				f=PGMATRIX_DATA(pU,i,i);
				// SIGN(a,b) ((b) >= 0.0 ? fabs(a) : -fabs(a))
				// g = -SIGN(sqrt(s),f);
				g = (GMATRIX_FLOAT)(-1.0 * sqrt(s) * GMATRIXMACRO_SIGN(f));
				h = f*g-s;
				PGMATRIX_DATA(pU,i,i)=f-g;
				for (j=l;j<=pMat->Nc;j++) {
					for (s=0.0,k=i;k<=pMat->Nr;k++) s += PGMATRIX_DATA(pU,k,i)*PGMATRIX_DATA(pU,k,j);
					f=s/h;
					for (k=i;k<=pMat->Nr;k++) PGMATRIX_DATA(pU,k,j) += f*PGMATRIX_DATA(pU,k,i);
				}
				for (k=i;k<=pMat->Nr;k++) PGMATRIX_DATA(pU,k,i) *= scale;
			}
		}
		PGMATRIX_DATA(pS,i,i) = scale * g;
		g=s=scale=0.0;
		if (i <= pMat->Nr && i != pMat->Nc) {
			for (k=l;k<=pMat->Nc;k++) scale += (GMATRIX_FLOAT)(fabs(PGMATRIX_DATA(pU,i,k)));
			if (scale) {
				for (k=l;k<=pMat->Nc;k++) {
					PGMATRIX_DATA(pU,i,k) /= scale;
					s += PGMATRIX_DATA(pU,i,k)*PGMATRIX_DATA(pU,i,k);
				}
				f=PGMATRIX_DATA(pU,i,l);
				// SIGN(a,b) ((b) >= 0.0 ? fabs(a) : -fabs(a))
				// g = -SIGN(sqrt(s),f);
				g = (GMATRIX_FLOAT)(-1.0 * sqrt(s) * GMATRIXMACRO_SIGN(f));
				h=f*g-s;
				PGMATRIX_DATA(pU,i,l)=f-g;
				for (k=l;k<=pMat->Nc;k++) PGMATRIX_DATA(pRV1,k,1)=PGMATRIX_DATA(pU,i,k)/h;
				for (j=l;j<=pMat->Nr;j++) {
					for (s=0.0,k=l;k<=pMat->Nc;k++) s += PGMATRIX_DATA(pU,j,k)*PGMATRIX_DATA(pU,i,k);
					for (k=l;k<=pMat->Nc;k++) PGMATRIX_DATA(pU,j,k) += s*PGMATRIX_DATA(pRV1,k,1);
				}
				for (k=l;k<=pMat->Nc;k++) PGMATRIX_DATA(pU,i,k) *= scale;
			}
		}
		anorm = GMATRIXMACRO_MAX(anorm,((GMATRIX_FLOAT)(fabs(PGMATRIX_DATA(pS,i,i))+fabs(PGMATRIX_DATA(pRV1,i,1)))));
	}
	for (i=pMat->Nc;i>=1;i--) {
		if (i < pMat->Nc) {
			if (g) {
				for (j=l;j<=pMat->Nc;j++) PGMATRIX_DATA(pV,j,i)=(PGMATRIX_DATA(pU,i,j)/PGMATRIX_DATA(pU,i,l))/g;
				for (j=l;j<=pMat->Nc;j++) {
					for (s=0.0,k=l;k<=pMat->Nc;k++) s += PGMATRIX_DATA(pU,i,k)*PGMATRIX_DATA(pV,k,j);
					for (k=l;k<=pMat->Nc;k++) PGMATRIX_DATA(pV,k,j) += s*PGMATRIX_DATA(pV,k,i);
				}
			}
			for (j=l;j<=pMat->Nc;j++) PGMATRIX_DATA(pV,i,j)=PGMATRIX_DATA(pV,j,i)=0.0;
		}
		PGMATRIX_DATA(pV,i,i)=1.0;
		g=PGMATRIX_DATA(pRV1,i,1);
		l=i;
	}
	for (i=GMATRIXMACRO_MIN(pMat->Nr,pMat->Nc);i>=1;i--) {
		l=i+1;
		g=PGMATRIX_DATA(pS,i,i);
		for (j=l;j<=pMat->Nc;j++) PGMATRIX_DATA(pU,i,j)=0.0;
		if (g) {
			g=(GMATRIX_FLOAT)(1.0)/g;
			for (j=l;j<=pMat->Nc;j++) {
				for (s=0.0,k=l;k<=pMat->Nr;k++) s += PGMATRIX_DATA(pU,k,i)*PGMATRIX_DATA(pU,k,j);
				f=(s/PGMATRIX_DATA(pU,i,i))*g;
				for (k=i;k<=pMat->Nr;k++) PGMATRIX_DATA(pU,k,j) += f*PGMATRIX_DATA(pU,k,i);
			}
			for (j=i;j<=pMat->Nr;j++) PGMATRIX_DATA(pU,j,i) *= g;
		} else for (j=i;j<=pMat->Nr;j++) PGMATRIX_DATA(pU,j,i)=0.0;
		++PGMATRIX_DATA(pU,i,i);
	}
	for (k=pMat->Nc;k>=1;k--) {
		for (its=1;its<=30;its++) {
			flag=1;
			for (l=k;l>=1;l--) {
				nm=l-1;
				if ((GMATRIX_FLOAT)(fabs(PGMATRIX_DATA(pRV1,l,1))+anorm) == anorm) {
					flag=0;
					break;
				}
				if ((GMATRIX_FLOAT)(fabs(PGMATRIX_DATA(pS,nm,nm))+anorm) == anorm) break;
			}
			if (flag) {
				c=0.0;
				s=1.0;
				for (i=l;i<=k;i++) {
					f=s*PGMATRIX_DATA(pRV1,i,1);
					PGMATRIX_DATA(pRV1,i,1)=c*PGMATRIX_DATA(pRV1,i,1);
					if ((GMATRIX_FLOAT)(fabs(f)+anorm) == anorm) break;
					g=PGMATRIX_DATA(pS,i,i);
					h=dpythag(f,g);
					PGMATRIX_DATA(pS,i,i)=h;
					h=(GMATRIX_FLOAT)(1.0)/h;
					c=g*h;
					s = -f*h;
					for (j=1;j<=pMat->Nr;j++) {
						y=PGMATRIX_DATA(pU,j,nm);
						z=PGMATRIX_DATA(pU,j,i);
						PGMATRIX_DATA(pU,j,nm)=y*c+z*s;
						PGMATRIX_DATA(pU,j,i)=z*c-y*s;
					}
				}
			}
			z=PGMATRIX_DATA(pS,k,k);
			if (l == k) {
				if (z < 0.0) {
					PGMATRIX_DATA(pS,k,k) = -z;
					for (j=1;j<=pMat->Nc;j++) PGMATRIX_DATA(pV,j,k) = -PGMATRIX_DATA(pV,j,k);
				}
				break;
			}
			if (its >= 30){
				GMATRIX_ASSERT("PGMATRIX_SVD: no convergence in 30 iterations",1);	
			}

			x=PGMATRIX_DATA(pS,l,l);
			nm=k-1;
			y=PGMATRIX_DATA(pS,nm,nm);
			g=PGMATRIX_DATA(pRV1,nm,1);
			h=PGMATRIX_DATA(pRV1,k,1);
			f=((y-z)*(y+z)+(g-h)*(g+h))/((GMATRIX_FLOAT)(2.0)*h*y);
			g=dpythag(f,1.0);
			// SIGN(a,b) ((b) >= 0.0 ? fabs(a) : -fabs(a))
			// f=((x-z)*(x+z)+h*((y/(f+SIGN(g,f)))-h))/x;
			f=(GMATRIX_FLOAT)(((x-z)*(x+z)+h*((y/(f+g*GMATRIXMACRO_SIGN(f)))-h))/x);
			c=s=1.0;
			for (j=l;j<=nm;j++) {
				i=j+1;
				g=PGMATRIX_DATA(pRV1,i,1);
				y=PGMATRIX_DATA(pS,i,i);
				h=s*g;
				g=c*g;
				z=dpythag(f,h);
				PGMATRIX_DATA(pRV1,j,1)=z;
				c=f/z;
				s=h/z;
				f=x*c+g*s;
				g = g*c-x*s;
				h=y*s;
				y *= c;
				for (jj=1;jj<=pMat->Nc;jj++) {
					x=PGMATRIX_DATA(pV,jj,j);
					z=PGMATRIX_DATA(pV,jj,i);
					PGMATRIX_DATA(pV,jj,j)=x*c+z*s;
					PGMATRIX_DATA(pV,jj,i)=z*c-x*s;
				}
				z=dpythag(f,h);
				PGMATRIX_DATA(pS,j,j)=z;
				if (z) {
					z=(GMATRIX_FLOAT)(1.0)/z;
					c=f*z;
					s=h*z;
				}
				f=c*g+s*y;
				x=c*y-s*g;
				for (jj=1;jj<=pMat->Nr;jj++) {
					y=PGMATRIX_DATA(pU,jj,j);
					z=PGMATRIX_DATA(pU,jj,i);
					PGMATRIX_DATA(pU,jj,j)=y*c+z*s;
					PGMATRIX_DATA(pU,jj,i)=z*c-y*s;
				}
			}
			PGMATRIX_DATA(pRV1,l,1)=0.0;
			PGMATRIX_DATA(pRV1,k,1)=f;
			PGMATRIX_DATA(pS,k,k)=x;
		}
	}
	PGMATRIX_FREE(pRV1);

	// Sort singular value matrix in descending order if requested:
	if((FlagSorted!=0) && (pS->Nr>1)){
		pcU = PGMATRIX_ALLOC(pU->Nr,1);
		pcV = PGMATRIX_ALLOC(pV->Nr,1);

		l = (pS->Nr >> 1) + 1;
		ir = pS->Nr;
		while(1){
			if (l > 1) {
				--l;
				cS = PGMATRIX_DATA(pS,l,l);
				PGMATRIX_COPY_COLUMN(pcU,1,pU,l);
				PGMATRIX_COPY_COLUMN(pcV,1,pV,l);
			} 
			else {
				cS = PGMATRIX_DATA(pS,ir,ir);
				PGMATRIX_COPY_COLUMN(pcU,1,pU,ir);
				PGMATRIX_COPY_COLUMN(pcV,1,pV,ir);

				PGMATRIX_DATA(pS,ir,ir) = PGMATRIX_DATA(pS,1,1);
				PGMATRIX_COPY_COLUMN(pU,ir,pU,1);
				PGMATRIX_COPY_COLUMN(pV,ir,pV,1);
				if (--ir == 1) {
					PGMATRIX_DATA(pS,1,1) = cS;
					PGMATRIX_COPY_COLUMN(pU,1,pcU,1);
					PGMATRIX_COPY_COLUMN(pV,1,pcV,1);
					break;
				}
			}
			i=l;
			j=l+l;
			while (j <= ir) {
				if (j < ir && PGMATRIX_DATA(pS,j,j) > PGMATRIX_DATA(pS,j+1,j+1)) j++;
				if (cS > PGMATRIX_DATA(pS,j,j)) {
					PGMATRIX_DATA(pS,i,i) = PGMATRIX_DATA(pS,j,j);
					PGMATRIX_COPY_COLUMN(pU,i,pU,j);
					PGMATRIX_COPY_COLUMN(pV,i,pV,j);
					i=j;
					j <<= 1;
				} else j=ir+1;
			}
			PGMATRIX_DATA(pS,i,i)=cS;
			PGMATRIX_COPY_COLUMN(pU,i,pcU,1);
			PGMATRIX_COPY_COLUMN(pV,i,pcV,1);
		}

		PGMATRIX_FREE(pcU);
		PGMATRIX_FREE(pcV);
	}
}
#endif

/**********************************************************************************************
***** GMatrix: Pseudoinverse using SVD:
***** Abstract: Apinv is the pseudoinverse of A.
***** Note: Apinv == NULL, put result on A matrix.
**********************************************************************************************/
#if GMATRIX_DEFINE_PSEUDOINVERSE
void PGMATRIX_PSEUDOINVERSE(PGMATRIX pApinv, PGMATRIX pA, PGMATRIX pMatDummy)
{
	int n;
	GMATRIX_FLOAT tol, norm;
	PGMATRIX pU;
	PGMATRIX pS;
	PGMATRIX pV;

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
	if (pApinv==NULL){
		PGMATRIX_MULTIPLY_COPY_EXTENDED(pA,pV,FALSE,pMatDummy,FALSE);
	}
	else{
		PGMATRIX_MULTIPLY_COPY_EXTENDED(pApinv,pV,FALSE,pMatDummy,FALSE);
	}

	PGMATRIX_FREE(pU);
	PGMATRIX_FREE(pS);
	PGMATRIX_FREE(pV);
}
#endif

/**********************************************************************************************
***** GMatrix: Moore-Penrose Pseudoinverses
**********************************************************************************************/
#if GMATRIX_DEFINE_LEFT_PSEUDOINVERSE_COPY
void PGMATRIX_LEFT_PSEUDOINVERSE_COPY(PGMATRIX pApinv, PGMATRIX pA, PGMATRIX pMatDummy)
{
	GMATRIX_ASSERT("PGMATRIX_DERIVATIVE_LEFT_PSEUDOINVERSE",pApinv->Nr!=pA->Nc);	
	GMATRIX_ASSERT("PGMATRIX_DERIVATIVE_LEFT_PSEUDOINVERSE",pApinv->Nc!=pA->Nr);	

	// Spinv = inv(S'*S)*S'
	PGMATRIX_MULTIPLY_COPY_EXTENDED(pMatDummy, pA, 1, pA, 0);
	PGMATRIX_INVERSE(pMatDummy);
	PGMATRIX_MULTIPLY_COPY_EXTENDED(pApinv, pMatDummy, 0, pA, 1);
}
#endif

/**********************************************************************************************
***** GMatrix: Derivative computations
**********************************************************************************************/
#if GMATRIX_DEFINE_DERIVATIVE_LEFT_PSEUDOINVERSE
void PGMATRIX_DERIVATIVE_LEFT_PSEUDOINVERSE(PGMATRIX pdApinvdx, PGMATRIX pdAdx, PGMATRIX pA, PGMATRIX pApinv, PGMATRIX pMatDummy1, PGMATRIX pMatDummy2)
{
	GMATRIX_ASSERT("PGMATRIX_DERIVATIVE_LEFT_PSEUDOINVERSE",pdAdx->Nr!=pA->Nr);	
	GMATRIX_ASSERT("PGMATRIX_DERIVATIVE_LEFT_PSEUDOINVERSE",pdAdx->Nc!=pA->Nc);	
	GMATRIX_ASSERT("PGMATRIX_DERIVATIVE_LEFT_PSEUDOINVERSE",pApinv->Nr!=pA->Nc);	
	GMATRIX_ASSERT("PGMATRIX_DERIVATIVE_LEFT_PSEUDOINVERSE",pApinv->Nc!=pA->Nr);	
	PGMATRIX_SETSIZE(pdApinvdx,pApinv->Nr,pApinv->Nc);	

	// Apinv = inv(A'*A)*A'
	// dApinvdx = -inv(A'*A)*dAdx'*(A*Apinv-I)-Apinv*dAdx*Apinv.

	PGMATRIX_TRIPLEMULTIPLY_COPY(pdApinvdx,pApinv,pdAdx,pApinv,pMatDummy1); // dApinvdx = Apinv*dAdx*Apinv
	PGMATRIX_MULTIPLY_COPY(pMatDummy1,pA,pApinv); // MatDummy1 = A*Apinv;
	PGMATRIX_SUBSTRACT_IDENTITY(pMatDummy1); // MatDummy1 = A*Apinv-I;

	PGMATRIX_MULTIPLY_COPY_EXTENDED(pMatDummy2,pdAdx,1,pMatDummy1,0); // MatDummy2 = dAdx'*(A*Apinv-I);
	PGMATRIX_MULTIPLY_COPY_EXTENDED(pMatDummy1,pA,1,pA,0); // MatDummy1 = A'*A
	PGMATRIX_INVERSE(pMatDummy1); // MatDummy1 = inv(A'*A);
	PGMATRIX_MULTIPLY_ADD(pdApinvdx,pMatDummy1,pMatDummy2); // dApinvdx = inv(A'*A)*dAdx'*(A*Apinv-I) + Apinv*dAdx*Apinv;
	PGMATRIX_MULTIPLY_CONST(pdApinvdx,-1.0); // dApinvdx = -inv(A'*A)*dAdx'*(A*Apinv-I) - Apinv*dAdx*Apinv;
}
#endif

/**********************************************************************************************
***** GMatrix: Rank of a matrix
**********************************************************************************************/
#if GMATRIX_DEFINE_RANK_FROMSVD
int PGMATRIX_RANK_FROMSVD(PGMATRIX pU, PGMATRIX pS, PGMATRIX pV)
{
	GMATRIX_FLOAT tol,maxabs;
	int rank,i;

	GMATRIX_ASSERT("PGMATRIX_RANK",pS->Nr==0);	
	GMATRIX_ASSERT("PGMATRIX_RANK",pS->Nc==0);	

	// Compute tol:
	maxabs = -1.0;
	for(i=1;i<=pS->Nr;++i){
		maxabs = GMATRIXMACRO_MAX(maxabs,PGMATRIX_DATA(pS,i,i));
	}
	tol = GMATRIXMACRO_MAX(pU->Nr,pV->Nr) * maxabs * (GMATRIX_FLOAT)(1e-16);

	// Compute the rank:
	for(i=1,rank=0;i<=pS->Nr;++i){
		if(PGMATRIX_DATA(pS,i,i) > tol){
			++rank;
		}
	}

	return(rank);
}
#endif

#if GMATRIX_DEFINE_RANK
int PGMATRIX_RANK(PGMATRIX pMat)
{
	int rank;
	PGMATRIX pU;
	PGMATRIX pS;
	PGMATRIX pV;

	GMATRIX_ASSERT("PGMATRIX_RANK",pMat->Nr==0);	
	GMATRIX_ASSERT("PGMATRIX_RANK",pMat->Nc==0);	

	// Alloc matrices:
	pU = PGMATRIX_ALLOC(pMat->Nr,pMat->Nc);
	pS = PGMATRIX_ALLOC(pMat->Nc,pMat->Nc);
	pV = PGMATRIX_ALLOC(pMat->Nc,pMat->Nc);

	// Compute SVD:
	PGMATRIX_SVD(pU,pS,pV,pMat,FALSE);

	// Compute rank:
	rank = PGMATRIX_RANK_FROMSVD(pU,pS,pV);

	// Free matrices:
	PGMATRIX_FREE(pU);
	PGMATRIX_FREE(pS);
	PGMATRIX_FREE(pV);

	return(rank);
}
#endif
