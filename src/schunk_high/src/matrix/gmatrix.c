/**********************************************************************************************
***********************************************************************************************
***  File:			GMatrix.c
***	 Author:		Geovany Araujo Borges
***	 Contents:		Implementation of Matrix type and related functions for use in C and C++.
***					Basic fnctions
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
#include<schunk_high/gmatrix.h>
/**********************************************************************************************
***** GMatrix: Includes.
**********************************************************************************************/
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

/**********************************************************************************************
***** GMatrix: Dynamic allocation
**********************************************************************************************/
#if GMATRIX_DEFINE_ALLOC
PGMATRIX PGMATRIX_ALLOC(int Nr, int Nc)
{
	PGMATRIX pMatrix;

	pMatrix = (GMATRIX*) malloc(sizeof(GMATRIX));
	GMATRIX_ASSERT("PGMATRIX_ALLOC",pMatrix==NULL);	

	pMatrix->Data = (GMATRIX_FLOAT*) malloc((Nr)*(Nc)*(sizeof(GMATRIX_FLOAT)));
	GMATRIX_ASSERT("PGMATRIX_ALLOC",pMatrix->Data==NULL);	

	pMatrix->Nr = (Nr);
	pMatrix->Nc = (Nc);
	pMatrix->MaxSize = (Nr)*(Nc);

	return(pMatrix);
}

void PGMATRIX_FREE(PGMATRIX pMatrix)
{
	free(pMatrix->Data);
	free(pMatrix);
}
#endif

/**********************************************************************************************
***** GMatrix: Utilities
**********************************************************************************************/
/** 
 *  PGMATRIX_SETSIZE � uma fun��o...
 */

#if GMATRIX_DEFINE_SETSIZE
void PGMATRIX_SETSIZE(PGMATRIX pMat,int Nll, int Ncc)
{
	GMATRIX_ASSERT("PGMATRIX_SETSIZE",pMat->MaxSize < (Nll*Ncc));	
	pMat->Nr = Nll; pMat->Nc = Ncc; 
}
#endif

#if GMATRIX_DEFINE_PRINT
void PGMATRIX_PRINT_NAMED(char* NameString,PGMATRIX pMat)
{
	int i,j;

	GMATRIX_ASSERT("PGMATRIX_PRINT",pMat == NULL);	
	GMATRIX_PRINTCOMMAND("\n%s (%i x %i):",NameString,pMat->Nr,pMat->Nc);	
	for(i=1;i<=pMat->Nr;++i){	
		GMATRIX_PRINTCOMMAND("\n   ");	
		for(j=1;j<=pMat->Nc;++j){	
			GMATRIX_PRINTCOMMAND("%c%4.14f ",GMATRIXMACRO_SIGNCHAR(PGMATRIX_DATA(pMat,i,j)),fabs(PGMATRIX_DATA(pMat,i,j)));	
		}	
	}
	GMATRIX_PRINTCOMMAND("\n");	
}
#endif

#if GMATRIX_DEFINE_PRINT_MATLABFORM
void PGMATRIX_PRINT_MATLABFORM_NAMED(char* NameString,PGMATRIX pMat)
{
	int i,j,nc,ncmax,columninc = 4,n;

	GMATRIX_ASSERT("PGMATRIX_PRINT_MATLABFORM",pMat == NULL);	
	GMATRIX_PRINTCOMMAND("\n%s = ",NameString);
	nc = 1;
	while (nc<=pMat->Nc){
		ncmax = nc + columninc - 1;
		if (ncmax >	pMat->Nc){
			ncmax = pMat->Nc;
		}
		GMATRIX_PRINTCOMMAND("\n  Columns %i through %i\n  ",nc,ncmax);
		for(i=1;i<=pMat->Nr;++i){
			for(j=nc;j<=ncmax;++j){
				if (fabs(PGMATRIX_DATA(pMat,i,j))>=1.0){
					GMATRIX_PRINTCOMMAND(" ");
				}
				if (fabs(PGMATRIX_DATA(pMat,i,j))>0.0){
					for(n=0;n<(4-log10(fabs(PGMATRIX_DATA(pMat,i,j))));++n) GMATRIX_PRINTCOMMAND(" ");
				}
				GMATRIX_PRINTCOMMAND("%c%4.14f ",GMATRIXMACRO_SIGNCHAR(PGMATRIX_DATA(pMat,i,j)),fabs(PGMATRIX_DATA(pMat,i,j)));	
			}	
			if(i!=pMat->Nr){
				GMATRIX_PRINTCOMMAND("\n  ");
			}
		}
		nc += columninc;
	}
	GMATRIX_PRINTCOMMAND("\n");	
}
#endif

#if GMATRIX_DEFINE_PRINT_EXP
void PGMATRIX_PRINT_EXP_NAMED(char* NameString,PGMATRIX pMat)
{
	int i,j;

	GMATRIX_ASSERT("PGMATRIX_PRINT_EXP",pMat == NULL);	
	GMATRIX_PRINTCOMMAND("\n%s (%i x %i):",NameString,pMat->Nr,pMat->Nc);	
	for(i=1;i<=pMat->Nr;++i){	
		GMATRIX_PRINTCOMMAND("\n   ");	
		for(j=1;j<=pMat->Nc;++j){	
			GMATRIX_PRINTCOMMAND("%c%4.14e ",GMATRIXMACRO_SIGNCHAR(PGMATRIX_DATA(pMat,i,j)),fabs(PGMATRIX_DATA(pMat,i,j)));	
		}	
	}
	GMATRIX_PRINTCOMMAND("\n");	
}
#endif

#if GMATRIX_DEFINE_PRINTROW
void PGMATRIX_PRINTROW_NAMED(char* NameString,PGMATRIX pMat,int i)
{
	int j;

	GMATRIX_ASSERT("PGMATRIX_PRINTROW",pMat == NULL);	
	GMATRIX_PRINTCOMMAND("\nRow %i of %s (%i x %i):",i,NameString,pMat->Nr,pMat->Nc);	
	GMATRIX_PRINTCOMMAND("\n   ");	
	for(j=1;j<=pMat->Nc;++j){	
		GMATRIX_PRINTCOMMAND("%c%4.5f ",GMATRIXMACRO_SIGNCHAR(PGMATRIX_DATA(pMat,i,j)),fabs(PGMATRIX_DATA(pMat,i,j)));	
	}
	GMATRIX_PRINTCOMMAND("\n");	
}
#endif

#if GMATRIX_DEFINE_INFO
void PGMATRIX_INFO_NAMED(char* NameString,PGMATRIX pMat)
{
	GMATRIX_PRINTCOMMAND("\n%s: Nr = %i, Nc = %i, MaxSize = %i",NameString,pMat->Nr,pMat->Nc,pMat->MaxSize);	
}
#endif

/**********************************************************************************************
***** GMatrix: Special Matrices
**********************************************************************************************/
#if GMATRIX_DEFINE_ZEROES
void PGMATRIX_ZEROES(PGMATRIX pMat)
{
	int i,j;
	for(i=1;i<=pMat->Nr;++i){	
		for(j=1;j<=pMat->Nc;++j){	
			PGMATRIX_DATA(pMat,i,j) = 0.0;	
		}	
	}
}
#endif

#if GMATRIX_DEFINE_SUMLINCOL
void PGMATRIX_SUMLINCOL(PGMATRIX pMat)
{
	int i,j;
	for(i=1;i<=pMat->Nr;++i){	
		for(j=1;j<=pMat->Nc;++j){	
			PGMATRIX_DATA(pMat,i,j) = (GMATRIX_FLOAT) i+j;	
		}	
	}
}
#endif

#if GMATRIX_DEFINE_ONES
void PGMATRIX_ONES(PGMATRIX pMat)
{
	int i,j;
	for(i=1;i<=pMat->Nr;++i){	
		for(j=1;j<=pMat->Nc;++j){	
			PGMATRIX_DATA(pMat,i,j) = 1.0;	
		}	
	}
}
#endif

#if GMATRIX_DEFINE_RAND
void PGMATRIX_RAND(PGMATRIX pMat)
{
	int i,j;
	for(i=1;i<=pMat->Nr;++i){	
		for(j=1;j<=pMat->Nc;++j){	
			PGMATRIX_DATA(pMat,i,j) = (((GMATRIX_FLOAT)(rand()))/RAND_MAX);	
		}	\
	}
}
#endif

#if GMATRIX_DEFINE_RANDN
void PGMATRIX_RANDN(PGMATRIX pMat)
{
	int i,j,k;
	for(i=1;i<=pMat->Nr;++i){	
		for(j=1;j<=pMat->Nc;++j){	
			PGMATRIX_DATA(pMat,i,j) = 0.0;
			for(k=1;k<=12;++k){	
				PGMATRIX_DATA(pMat,i,j) += ((((GMATRIX_FLOAT)(rand()))/RAND_MAX)-((GMATRIX_FLOAT)(0.5)));	
			}
		}	
	}
}
#endif

#if GMATRIX_DEFINE_IDENTITY
void PGMATRIX_IDENTITY(PGMATRIX pMat) 
{
	int i,j;
	GMATRIX_ASSERT("PGMATRIX_IDENTITY",pMat->Nr != pMat->Nc);	
	for(i=1;i<=pMat->Nr;++i){	
		for(j=1;j<=pMat->Nc;++j){	
			PGMATRIX_DATA(pMat,i,j) = 0.0;	
		}	
		PGMATRIX_DATA(pMat,i,i) = 1.0;	
	}
}
#endif

#if GMATRIX_DEFINE_WILKINSON
void PGMATRIX_WILKINSON(PGMATRIX pMat)
{
	int m,i,j;

	GMATRIX_ASSERT("PGMATRIX_WILKINSON",pMat->Nr != pMat->Nc);	
	GMATRIX_ASSERT("In PGMATRIX_WILKINSON, the matrix must have an odd number of columns",(pMat->Nr % 2) == 0);	

	// m = (n-1)/2;
	m = (pMat->Nr-1)/2;
	// e = ones(n-1,1);
	// W = diag(abs(-m:m)) + diag(e,1) + diag(e,-1);
	PGMATRIX_ZEROES(pMat);
	for(i=1;i<=pMat->Nr;++i){	
		for(j=1;j<=pMat->Nc;++j){	
			if (i==j){
				PGMATRIX_DATA(pMat,i,i) = (GMATRIX_FLOAT) fabs(i-m-1);
			}
			else{
				if ((i==j-1) && (j > 1) ){
					PGMATRIX_DATA(pMat,i,j) = 1;
				}
				if ((j==i-1) && (i > 1) ){
					PGMATRIX_DATA(pMat,i,j) = 1;
				}
			}
		}
	}
}
#endif

#if GMATRIX_DEFINE_ORTHOGONAL
void PGMATRIX_ORTHOGONAL(PGMATRIX pMat,int Type) 
{
	/* Orthogonal matrix according to Type and the matrix order N:
    Type = 1:  Q(i,j) = SQRT(2/(N+1)) * SIN( i*j*PI/(N+1) )
            Symmetric eigenvector matrix for second difference matrix.
            This is the default Type.
    Type = 2:  Q(i,j) = 2/(SQRT(2*N+1)) * SIN( 2*i*j*PI/(2*N+1) )
            Symmetric.
    Type = -1: Q(i,j) = COS( (i-1)*(j-1)*PI/(N-1) )
            Chebyshev Vandermonde-like matrix, based on extrema 
            of T(N-1).
    Type = -2: Q(i,j) = COS( (i-1)*(j-1/2)*PI/N) )
            Chebyshev Vandermonde-like matrix, based on zeros of T(N).*/
	// Extracted from gallery MATLAB function

	GMATRIX_FLOAT N;
	int i,j;

	N = (GMATRIX_FLOAT)(pMat->Nr);
	GMATRIX_ASSERT("PGMATRIX_ORTHOGONAL",pMat->Nr != pMat->Nc);	
	GMATRIX_ASSERT("PGMATRIX_ORTHOGONAL, Type is not a valid value",(Type!=1)&&(Type!=2)&&(Type!=-1)&&(Type!=-2));	
	PGMATRIX_ZEROES(pMat);
	for(i=1;i<=pMat->Nr;++i){	
		for(j=1;j<=pMat->Nc;++j){	
			switch(Type){
			case 1:
				PGMATRIX_DATA(pMat,i,j) = (GMATRIX_FLOAT)(sqrt(2.0/(N+1.0)) * sin( i*j*GMATRIXCONST_PI/(N+1.0) ));
				break;
			case 2:
				PGMATRIX_DATA(pMat,i,j) = (GMATRIX_FLOAT)(2.0/(sqrt(2.0*N+1.0)) * sin( 2.0*i*j*GMATRIXCONST_PI/(2.0*N+1.0) ));
				break;
			case -1:
				PGMATRIX_DATA(pMat,i,j) = (GMATRIX_FLOAT)(cos( (i-1.0)*(j-1.0)*GMATRIXCONST_PI/(N-1.0) ));
				break;
			case -2:
				// m = (0:n-1)'*(.5:n-.5) * (pi/n)
				PGMATRIX_DATA(pMat,i,j) = (GMATRIX_FLOAT)(cos( (i-1.0)*(j-1.0/2.0)*GMATRIXCONST_PI/N ));
				break;
			}
		}
	}
}
#endif

/**********************************************************************************************
***** GMatrix: COPY
**********************************************************************************************/
#if GMATRIX_DEFINE_COPY
void PGMATRIX_COPY(PGMATRIX pMatResult, PGMATRIX pMat)
{
	int i,j;
	GMATRIX_ASSERT("PGMATRIX_COPY",pMatResult->MaxSize < (pMat->Nr*pMat->Nc));	
	PGMATRIX_SETSIZE(pMatResult,pMat->Nr,pMat->Nc);
	for(i=1;i<=pMatResult->Nr;++i){	
		for(j=1;j<=pMatResult->Nc;++j){	
			PGMATRIX_DATA(pMatResult,i,j) = PGMATRIX_DATA(pMat,i,j); 
		}	
	}
}
#endif

#if GMATRIX_DEFINE_COPY_COLUMN
void PGMATRIX_COPY_COLUMN(PGMATRIX pMatResult, int ColDest, PGMATRIX pMat, int ColOrigin)
{
	int i;
	GMATRIX_ASSERT("PGMATRIX_COPY_COLUMN",pMatResult->Nr != pMat->Nr);	
	for(i=1;i<=pMatResult->Nr;++i){	
		PGMATRIX_DATA(pMatResult,i,ColDest) = PGMATRIX_DATA(pMat,i,ColOrigin); 
	}
}
#endif

#if GMATRIX_DEFINE_COPY_ROW
void PGMATRIX_COPY_ROW(PGMATRIX pMatResult, int RowDest, PGMATRIX pMat, int RowOrigin)
{
	int i;
	GMATRIX_ASSERT("PGMATRIX_COPY_ROW",pMatResult->Nc != pMat->Nc);	
	for(i=1;i<=pMatResult->Nc;++i){	
		PGMATRIX_DATA(pMatResult,RowDest,i) = PGMATRIX_DATA(pMat,RowOrigin,i); 
	}
}
#endif

/**********************************************************************************************
***** GMatrix: Transposition
**********************************************************************************************/
#if GMATRIX_DEFINE_TRANSPOSE_COPY
void PGMATRIX_TRANSPOSE_COPY(PGMATRIX pMatTranspose, PGMATRIX pMat) 
{
	int i,j;

	PGMATRIX_SETSIZE(pMatTranspose,pMat->Nc,pMat->Nr);	

	for(i=1;i<=pMat->Nr;++i){	
		for(j=1;j<=pMat->Nc;++j){	
			PGMATRIX_DATA(pMatTranspose,j,i) = PGMATRIX_DATA(pMat,i,j);	
		}	
	}
}
#endif

/**********************************************************************************************
***** GMatrix: Norm
**********************************************************************************************/
#if GMATRIX_DEFINE_NORM
GMATRIX_FLOAT PGMATRIX_NORM(PGMATRIX pMat)
{
  GMATRIX_FLOAT sqrNorm = 0.0;
  int i,j;
  
  for(i=1;i<=pMat->Nr;++i){	
		for(j=1;j<=pMat->Nc;++j){	
			sqrNorm = sqrNorm + PGMATRIX_DATA(pMat,i,j)*PGMATRIX_DATA(pMat,i,j);	
		}	
	}

  return sqrt(sqrNorm);
}

#endif

/**********************************************************************************************
***** GMatrix: Trace
**********************************************************************************************/
#if GMATRIX_DEFINE_TRACE
GMATRIX_FLOAT PGMATRIX_TRACE(PGMATRIX pMat)
{
	GMATRIX_FLOAT Trace;

	int i;
	GMATRIX_ASSERT("GMATRIX_TRACE",pMat->Nr != pMat->Nc);

	for (i=1,Trace=0.0; i<=pMat->Nr; ++i){ 
		Trace += PGMATRIX_DATA(pMat,i,i); 
	} 

	return Trace;
}
#endif

/**********************************************************************************************
***** GMatrix: Sum entries
**********************************************************************************************/
#if GMATRIX_DEFINE_SUMENTRIES
GMATRIX_FLOAT PGMATRIX_SUMENTRIES(PGMATRIX pMat)
{
	GMATRIX_FLOAT Sum = 0.0;
	int i,j;

	for (i=1; i<=pMat->Nr; ++i){ 
		for (j=1; j<=pMat->Nc; ++j){ 
			Sum += PGMATRIX_DATA(pMat,i,j); 
		}
	} 

	return Sum;
}
#endif

/**********************************************************************************************
***** GMatrix: Sum absolute entries
**********************************************************************************************/
#if GMATRIX_DEFINE_SUMABSOLUTEENTRIES
GMATRIX_FLOAT PGMATRIX_SUMABSOLUTEENTRIES(PGMATRIX pMat)
{
	GMATRIX_FLOAT Sum = 0.0;
	int i,j;

	for (i=1; i<=pMat->Nr; ++i){ 
		for (j=1; j<=pMat->Nc; ++j){ 
			Sum += (GMATRIX_FLOAT)(fabs(PGMATRIX_DATA(pMat,i,j))); 
		}
	} 

	return Sum;
}
#endif

/**********************************************************************************************
***** GMatrix: Row and Column swapping
**********************************************************************************************/
#if GMATRIX_DEFINE_SWAP_ROW
void PGMATRIX_SWAP_ROW(PGMATRIX pMat,int i, int j)
{
	GMATRIX_FLOAT temp;
	int k;

	GMATRIX_ASSERT("PGMATRIX_SWAP_ROW",pMat->Nr <= 0);
	GMATRIX_ASSERT("PGMATRIX_SWAP_ROW",pMat->Nr < i);
	GMATRIX_ASSERT("PGMATRIX_SWAP_ROW",pMat->Nr < j);
	GMATRIX_ASSERT("PGMATRIX_SWAP_ROW",i <= 0);
	GMATRIX_ASSERT("PGMATRIX_SWAP_ROW",j <= 0);

	if(i==j) return;
	for(k=1;k<=pMat->Nc;++k){
		GMATRIXMACRO_SWAP(PGMATRIX_DATA(pMat,i,k),PGMATRIX_DATA(pMat,j,k));
	}
}
#endif

#if GMATRIX_DEFINE_SWAP_COLUMN
void PGMATRIX_SWAP_COLUMN(PGMATRIX pMat,int i, int j)
{
	GMATRIX_FLOAT temp;
	int k;

	GMATRIX_ASSERT("PGMATRIX_SWAP_COLUMN",pMat->Nc <= 0);
	GMATRIX_ASSERT("PGMATRIX_SWAP_COLUMN",pMat->Nc < i);
	GMATRIX_ASSERT("PGMATRIX_SWAP_COLUMN",pMat->Nc < j);
	GMATRIX_ASSERT("PGMATRIX_SWAP_COLUMN",i <= 0);
	GMATRIX_ASSERT("PGMATRIX_SWAP_COLUMN",j <= 0);

	if(i==j) return;
	for(k=1;k<=pMat->Nr;++k){
		GMATRIXMACRO_SWAP(PGMATRIX_DATA(pMat,k,i),PGMATRIX_DATA(pMat,k,j));
	}
}
#endif


/**********************************************************************************************
***** GMatrix: Arithmetic with constants
**********************************************************************************************/
#if GMATRIX_DEFINE_ADD_CONST
void PGMATRIX_ADD_CONST(PGMATRIX pMat,GMATRIX_FLOAT Value)
{
	int i,j;
	for(i=1;i<=pMat->Nr;++i){	
		for(j=1;j<=pMat->Nc;++j){	
			PGMATRIX_DATA(pMat,i,j) += Value;	
		}	
	}
}
#endif

#if GMATRIX_DEFINE_MULTIPLY_CONST
void PGMATRIX_MULTIPLY_CONST(PGMATRIX pMat,GMATRIX_FLOAT Value)
{
	int i,j;
	for(i=1;i<=pMat->Nr;++i){	
		for(j=1;j<=pMat->Nc;++j){	
			PGMATRIX_DATA(pMat,i,j) *= Value;	
		}	
	}
}
#endif

#if GMATRIX_DEFINE_MULTIPLY_CONST_COPY
void PGMATRIX_MULTIPLY_CONST_COPY(PGMATRIX pMatResult, PGMATRIX pMat,GMATRIX_FLOAT Value)
{
	int i,j;
	PGMATRIX_SETSIZE(pMatResult,pMat->Nr,pMat->Nc);
	for(i=1;i<=pMat->Nr;++i){	
		for(j=1;j<=pMat->Nc;++j){	
			PGMATRIX_DATA(pMatResult,i,j) =  PGMATRIX_DATA(pMat,i,j) * Value;
		}	
	}
}
#endif

#if GMATRIX_DEFINE_MULTIPLY_CONST_ADD
void PGMATRIX_MULTIPLY_CONST_ADD(PGMATRIX pMatResult, PGMATRIX pMat,GMATRIX_FLOAT Value)
{
	int i,j;
	GMATRIX_ASSERT("PGMATRIX_MULTIPLY_CONST_ADD",pMatResult->Nr != pMat->Nr);	
	GMATRIX_ASSERT("PGMATRIX_MULTIPLY_CONST_ADD",pMatResult->Nc != pMat->Nc);	
	for(i=1;i<=pMat->Nr;++i){	
		for(j=1;j<=pMat->Nc;++j){	
			PGMATRIX_DATA(pMatResult,i,j) +=  PGMATRIX_DATA(pMat,i,j) * Value;
		}	
	}
}
#endif

/**********************************************************************************************
***** GMatrix: Arithmetic with matrices
**********************************************************************************************/
#if GMATRIX_DEFINE_ADD_COPY
void PGMATRIX_ADD_COPY(PGMATRIX pMatResult, PGMATRIX pMatA, PGMATRIX pMatB)
{
	int i,j;
	GMATRIX_ASSERT("PGMATRIX_ADD_COPY",pMatA->Nr != pMatB->Nr);	
	GMATRIX_ASSERT("PGMATRIX_ADD_COPY",pMatA->Nc != pMatB->Nc);	
	PGMATRIX_SETSIZE(pMatResult,pMatA->Nr, pMatA->Nc);
	for(i=1;i<=pMatA->Nr;++i){	
		for(j=1;j<=pMatA->Nc;++j){	
			PGMATRIX_DATA(pMatResult,i,j) = PGMATRIX_DATA(pMatA,i,j) + PGMATRIX_DATA(pMatB,i,j);
		}	
	}
}
#endif

#if GMATRIX_DEFINE_ADD
void PGMATRIX_ADD(PGMATRIX pMatA, PGMATRIX pMatB)
{
	int i,j;
	GMATRIX_ASSERT("PGMATRIX_ADD",pMatA->Nr != pMatB->Nr);	
	GMATRIX_ASSERT("PGMATRIX_ADD",pMatA->Nc != pMatB->Nc);	
	for(i=1;i<=pMatB->Nr;++i){	
		for(j=1;j<=pMatB->Nc;++j){	
			PGMATRIX_DATA(pMatA,i,j) += PGMATRIX_DATA(pMatB,i,j);
		}	
	}
}
#endif

#if GMATRIX_DEFINE_SUBSTRACT_COPY
void PGMATRIX_SUBSTRACT_COPY(PGMATRIX pMatResult, PGMATRIX pMatA, PGMATRIX pMatB)
{
	int i,j;
	GMATRIX_ASSERT("GMATRIX_SUBSTRACT_COPY",pMatA->Nr != pMatB->Nr);	
	GMATRIX_ASSERT("GMATRIX_SUBSTRACT_COPY",pMatA->Nc != pMatB->Nc);	
	PGMATRIX_SETSIZE(pMatResult,pMatA->Nr, pMatA->Nc);
	for(i=1;i<=pMatA->Nr;++i){	
		for(j=1;j<=pMatA->Nc;++j){	
			PGMATRIX_DATA(pMatResult,i,j) = PGMATRIX_DATA(pMatA,i,j) - PGMATRIX_DATA(pMatB,i,j); 
		}	
	}
}
#endif

#if GMATRIX_DEFINE_SUBSTRACT_IDENTITY_COPY
void PGMATRIX_SUBSTRACT_IDENTITY_COPY(PGMATRIX pMatResult, PGMATRIX pMat)
{
	int i,j;
	GMATRIX_ASSERT("GMATRIX_SUBSTRACT_COPY",pMat->Nr != pMat->Nc);	
	PGMATRIX_SETSIZE(pMatResult,pMat->Nr, pMat->Nc);
	for(i=1;i<=pMat->Nr;++i){	
		for(j=1;j<=pMat->Nc;++j){
			if (j==i){
				PGMATRIX_DATA(pMatResult,i,j) = PGMATRIX_DATA(pMat,i,j) - (GMATRIX_FLOAT)(1.0);
			}
			else{
				PGMATRIX_DATA(pMatResult,i,j) = PGMATRIX_DATA(pMat,i,j);
			}
		}	
	}
}
#endif

#if GMATRIX_DEFINE_SUBSTRACT_IDENTITY
void PGMATRIX_SUBSTRACT_IDENTITY(PGMATRIX pMat)
{
	int i,j;
	GMATRIX_ASSERT("GMATRIX_SUBSTRACT_COPY",pMat->Nr != pMat->Nc);	
	for(i=1;i<=pMat->Nr;++i){	
		for(j=1;j<=pMat->Nc;++j){
			if (j==i){
				PGMATRIX_DATA(pMat,i,j) = PGMATRIX_DATA(pMat,i,j) - (GMATRIX_FLOAT)(1.0);
			}
		}	
	}
}
#endif

#if GMATRIX_DEFINE_SUBSTRACT
void PGMATRIX_SUBSTRACT(PGMATRIX pMatA, PGMATRIX pMatB)
{
	int i,j;
	GMATRIX_ASSERT("GMATRIX_SUBSTRACT_COPY",pMatA->Nr != pMatB->Nr);	
	GMATRIX_ASSERT("GMATRIX_SUBSTRACT_COPY",pMatA->Nc != pMatB->Nc);	
	for(i=1;i<=pMatA->Nr;++i){	
		for(j=1;j<=pMatA->Nc;++j){	
			PGMATRIX_DATA(pMatA,i,j) -= PGMATRIX_DATA(pMatB,i,j); 
		}	
	}
}
#endif

#if GMATRIX_DEFINE_CROSS
void PGMATRIX_CROSS_COPY(PGMATRIX pMatResult, PGMATRIX pMatA, PGMATRIX pMatB)
{

	GMATRIX_ASSERT("PGMATRIX_CROSS_COPY",pMatA->Nr != 3);
	GMATRIX_ASSERT("PGMATRIX_CROSS_COPY",pMatB->Nr != 3);
	GMATRIX_ASSERT("PGMATRIX_CROSS_COPY",pMatA->Nc != 1);
	GMATRIX_ASSERT("PGMATRIX_CROSS_COPY",pMatB->Nc != 1);
	PGMATRIX_SETSIZE(pMatResult,3, 1);



	// A2B3-A3B2
	PGMATRIX_DATA(pMatResult,1,1) = PGMATRIX_DATA(pMatA,2,1)*PGMATRIX_DATA(pMatB,3,1) - PGMATRIX_DATA(pMatA,3,1)*PGMATRIX_DATA(pMatB,2,1);
	// A3B1-A1B3
	PGMATRIX_DATA(pMatResult,2,1) = PGMATRIX_DATA(pMatA,3,1)*PGMATRIX_DATA(pMatB,1,1) - PGMATRIX_DATA(pMatA,1,1)*PGMATRIX_DATA(pMatB,3,1);
	// A1B2-A2B1
	PGMATRIX_DATA(pMatResult,3,1) = PGMATRIX_DATA(pMatA,1,1)*PGMATRIX_DATA(pMatB,2,1) - PGMATRIX_DATA(pMatA,2,1)*PGMATRIX_DATA(pMatB,1,1);
}

#endif

#if GMATRIX_DEFINE_MULTIPLY_COPY
void PGMATRIX_MULTIPLY_COPY(PGMATRIX pMatResult, PGMATRIX pMatA, PGMATRIX pMatB) 
{
	int i,j;
	int k;
	GMATRIX_FLOAT Acc;

	GMATRIX_ASSERT("PGMATRIX_MULTIPLY_COPY",pMatA->Nc != pMatB->Nr);	
	PGMATRIX_SETSIZE(pMatResult,pMatA->Nr, pMatB->Nc);
	for(i=1;i<=pMatA->Nr;++i){	
		for(j=1;j<=pMatB->Nc;++j){	
			Acc = 0.0;
			for(k=1;k<=pMatA->Nc;++k){	
				Acc += PGMATRIX_DATA(pMatA,i,k)*PGMATRIX_DATA(pMatB,k,j);	
			}	
			PGMATRIX_DATA(pMatResult,i,j) = Acc;	
		}	
	}
}
#endif

#if GMATRIX_DEFINE_MULTIPLY_COPY_EXTENDED
void PGMATRIX_MULTIPLY_COPY_EXTENDED(PGMATRIX pMatResult, PGMATRIX pMatA, int FlagTransposeA, PGMATRIX pMatB, int FlagTransposeB) 
{
	int k;
	int i,j;
	GMATRIX_FLOAT Acc;

	if ( !FlagTransposeA && !FlagTransposeB ){
		GMATRIX_ASSERT("PGMATRIX_MULTIPLY_COPY_EXTENDED",pMatA->Nc != pMatB->Nr);	
		PGMATRIX_SETSIZE(pMatResult,pMatA->Nr, pMatB->Nc);
		for(i=1;i<=pMatA->Nr;++i){	
			for(j=1;j<=pMatB->Nc;++j){	
				Acc = 0.0;
				for(k=1;k<=pMatA->Nc;++k){	
					Acc += PGMATRIX_DATA(pMatA,i,k)*PGMATRIX_DATA(pMatB,k,j);	
				}	
				PGMATRIX_DATA(pMatResult,i,j) = Acc;	
			}	
		}
	}
	if ( !FlagTransposeA && FlagTransposeB ){
		GMATRIX_ASSERT("PGMATRIX_MULTIPLY_COPY_EXTENDED (pMatB is transposed)",pMatA->Nc != pMatB->Nc);	
		PGMATRIX_SETSIZE(pMatResult,pMatA->Nr, pMatB->Nr);
		for(i=1;i<=pMatA->Nr;++i){	
			for(j=1;j<=pMatB->Nr;++j){	
				Acc = 0.0;
				for(k=1;k<=pMatA->Nc;++k){	
					Acc += PGMATRIX_DATA(pMatA,i,k)*PGMATRIX_DATA(pMatB,j,k);	
				}	
				PGMATRIX_DATA(pMatResult,i,j) = Acc;	
			}	
		}
	}
	if ( FlagTransposeA && !FlagTransposeB ){
		GMATRIX_ASSERT("PGMATRIX_MULTIPLY_COPY_EXTENDED (pMatA is transposed)",pMatA->Nr != pMatB->Nr);	
		PGMATRIX_SETSIZE(pMatResult,pMatA->Nc, pMatB->Nc);
		for(i=1;i<=pMatA->Nc;++i){	
			for(j=1;j<=pMatB->Nc;++j){	
				Acc = 0.0;
				for(k=1;k<=pMatA->Nr;++k){	
					Acc += PGMATRIX_DATA(pMatA,k,i)*PGMATRIX_DATA(pMatB,k,j);	
				}	
				PGMATRIX_DATA(pMatResult,i,j) = Acc;	
			}	
		}
	}
	if ( FlagTransposeA && FlagTransposeB ){
		GMATRIX_ASSERT("PGMATRIX_MULTIPLY_COPY_EXTENDED (pMatA and pMatB are transposed)",pMatA->Nr != pMatB->Nc);	
		PGMATRIX_SETSIZE(pMatResult,pMatA->Nc, pMatB->Nr);
		for(i=1;i<=pMatA->Nc;++i){	
			for(j=1;j<=pMatB->Nr;++j){	
				Acc = 0.0;
				for(k=1;k<=pMatA->Nr;++k){	
					Acc += PGMATRIX_DATA(pMatA,k,i)*PGMATRIX_DATA(pMatB,j,k);	
				}	
				PGMATRIX_DATA(pMatResult,i,j) = Acc;	
			}	
		}
	}
}
#endif

#if GMATRIX_DEFINE_MULTIPLY_ADD
void PGMATRIX_MULTIPLY_ADD(PGMATRIX pMatResult, PGMATRIX pMatA, PGMATRIX pMatB) 
{
	int k;
	int i,j;
	GMATRIX_FLOAT Acc;

	GMATRIX_ASSERT("PGMATRIX_MULTIPLY_ADD",pMatA->Nc != pMatB->Nr);	
	GMATRIX_ASSERT("PGMATRIX_MULTIPLY_ADD",pMatResult->Nr != pMatA->Nr);	
	GMATRIX_ASSERT("PGMATRIX_MULTIPLY_ADD",pMatResult->Nc != pMatB->Nc);	
	for(i=1;i<=pMatA->Nr;++i){	
		for(j=1;j<=pMatB->Nc;++j){	
			Acc = 0.0;
			for(k=1;k<=pMatA->Nc;++k){	
				Acc += PGMATRIX_DATA(pMatA,i,k)*PGMATRIX_DATA(pMatB,k,j);	
			}	
			PGMATRIX_DATA(pMatResult,i,j) += Acc;	
		}	
	}
}
#endif

#if GMATRIX_DEFINE_MULTIPLY_ADD_EXTENDED
void PGMATRIX_MULTIPLY_ADD_EXTENDED(PGMATRIX pMatResult, PGMATRIX pMatA, int FlagTransposeA, PGMATRIX pMatB, int FlagTransposeB) 

{

	int k;

	int i,j;

	double Acc;



	if ( !FlagTransposeA && !FlagTransposeB ){

		GMATRIX_ASSERT("PGMATRIX_MULTIPLY_COPY_EXTENDED",pMatA->Nc != pMatB->Nr);	

		PGMATRIX_SETSIZE(pMatResult,pMatA->Nr, pMatB->Nc);

		for(i=1;i<=pMatA->Nr;++i){	

			for(j=1;j<=pMatB->Nc;++j){	

				Acc = 0.0;

				for(k=1;k<=pMatA->Nc;++k){	

					Acc += PGMATRIX_DATA(pMatA,i,k)*PGMATRIX_DATA(pMatB,k,j);	

				}	

				PGMATRIX_DATA(pMatResult,i,j) += Acc;	

			}	

		}

	}

	if ( !FlagTransposeA && FlagTransposeB ){

		GMATRIX_ASSERT("PGMATRIX_MULTIPLY_COPY_EXTENDED (pMatB is transposed)",pMatA->Nc != pMatB->Nc);	

		PGMATRIX_SETSIZE(pMatResult,pMatA->Nr, pMatB->Nr);

		for(i=1;i<=pMatA->Nr;++i){	

			for(j=1;j<=pMatB->Nr;++j){	

				Acc = 0.0;

				for(k=1;k<=pMatA->Nc;++k){	

					Acc += PGMATRIX_DATA(pMatA,i,k)*PGMATRIX_DATA(pMatB,j,k);	

				}	

				PGMATRIX_DATA(pMatResult,i,j) += Acc;	

			}	

		}

	}

	if ( FlagTransposeA && !FlagTransposeB ){

		GMATRIX_ASSERT("PGMATRIX_MULTIPLY_COPY_EXTENDED (pMatA is transposed)",pMatA->Nr != pMatB->Nr);	

		PGMATRIX_SETSIZE(pMatResult,pMatA->Nc, pMatB->Nc);

		for(i=1;i<=pMatA->Nc;++i){	

			for(j=1;j<=pMatB->Nc;++j){	

				Acc = 0.0;

				for(k=1;k<=pMatA->Nr;++k){	

					Acc += PGMATRIX_DATA(pMatA,k,i)*PGMATRIX_DATA(pMatB,k,j);	

				}	

				PGMATRIX_DATA(pMatResult,i,j) += Acc;	

			}	

		}

	}

	if ( FlagTransposeA && FlagTransposeB ){

		GMATRIX_ASSERT("PGMATRIX_MULTIPLY_COPY_EXTENDED (pMatA and pMatB are transposed)",pMatA->Nr != pMatB->Nc);	

		PGMATRIX_SETSIZE(pMatResult,pMatA->Nc, pMatB->Nr);

		for(i=1;i<=pMatA->Nc;++i){	

			for(j=1;j<=pMatB->Nr;++j){	

				Acc = 0.0;

				for(k=1;k<=pMatA->Nr;++k){	

					Acc += PGMATRIX_DATA(pMatA,k,i)*PGMATRIX_DATA(pMatB,j,k);	

				}	

				PGMATRIX_DATA(pMatResult,i,j) += Acc;	

			}	

		}

	}

}

#endif

#if GMATRIX_DEFINE_MULTIPLY_SUBSTRACT
void PGMATRIX_MULTIPLY_SUBSTRACT(PGMATRIX pMatResult, PGMATRIX pMatA, PGMATRIX pMatB) 
{
	int k;
	int i,j;
	GMATRIX_FLOAT Acc;

	GMATRIX_ASSERT("PGMATRIX_MULTIPLY_SUBSTRACT",pMatA->Nc != pMatB->Nr);	
	GMATRIX_ASSERT("PGMATRIX_MULTIPLY_SUBSTRACT",pMatResult->Nr != pMatA->Nr);	
	GMATRIX_ASSERT("PGMATRIX_MULTIPLY_SUBSTRACT",pMatResult->Nc != pMatB->Nc);	
	for(i=1;i<=pMatA->Nr;++i){	
		for(j=1;j<=pMatB->Nc;++j){	
			Acc = 0.0;
			for(k=1;k<=pMatA->Nc;++k){	
				Acc += PGMATRIX_DATA(pMatA,i,k)*PGMATRIX_DATA(pMatB,k,j);	
			}	
			PGMATRIX_DATA(pMatResult,i,j) -= Acc;	
		}	
	}
}
#endif

#if GMATRIX_DEFINE_TRIPLEMULTIPLY_COPY
void PGMATRIX_TRIPLEMULTIPLY_COPY(PGMATRIX pMatResult, PGMATRIX pMatA, PGMATRIX pMatB, PGMATRIX pMatC, PGMATRIX pMatDummy) 
{
	GMATRIX_ASSERT("PGMATRIX_TRIPLEMULTIPLY_COPY",pMatA->Nc != pMatB->Nr);	
	GMATRIX_ASSERT("PGMATRIX_TRIPLEMULTIPLY_COPY",pMatB->Nc != pMatC->Nr);	
	PGMATRIX_MULTIPLY_COPY(pMatDummy, pMatB, pMatC);	
	PGMATRIX_MULTIPLY_COPY(pMatResult, pMatA, pMatDummy);
}
#endif

#if GMATRIX_DEFINE_TRIPLEMULTIPLY_COPY_EXTENDED
void PGMATRIX_TRIPLEMULTIPLY_COPY_EXTENDED(PGMATRIX pMatResult, PGMATRIX pMatA, BOOL FlagTransposeA, PGMATRIX pMatB, BOOL FlagTransposeB, PGMATRIX pMatC, BOOL FlagTransposeC, PGMATRIX pMatDummy) 
{
//	GMATRIX_ASSERT("PGMATRIX_TRIPLEMULTIPLY_COPY",pMatA->Nc != pMatB->Nr);	
//	GMATRIX_ASSERT("PGMATRIX_TRIPLEMULTIPLY_COPY",pMatB->Nc != pMatC->Nr);	
	PGMATRIX_MULTIPLY_COPY_EXTENDED(pMatDummy, pMatB, FlagTransposeB, pMatC, FlagTransposeC);	
	PGMATRIX_MULTIPLY_COPY_EXTENDED(pMatResult, pMatA, FlagTransposeA, pMatDummy, FALSE);
}
#endif

#if GMATRIX_DEFINE_TRIPLEMULTIPLY_ADD
void PGMATRIX_TRIPLEMULTIPLY_ADD(PGMATRIX pMatResult, PGMATRIX pMatA, PGMATRIX pMatB, PGMATRIX pMatC, PGMATRIX pMatDummy) 
{
	GMATRIX_ASSERT("PGMATRIX_TRIPLEMULTIPLY_ADD",pMatA->Nc != pMatB->Nr);	
	GMATRIX_ASSERT("PGMATRIX_TRIPLEMULTIPLY_ADD",pMatB->Nc != pMatC->Nr);	
	GMATRIX_ASSERT("PGMATRIX_TRIPLEMULTIPLY_ADD",pMatResult->Nr != pMatA->Nr); 
	GMATRIX_ASSERT("PGMATRIX_TRIPLEMULTIPLY_ADD",pMatResult->Nc != pMatC->Nc); 
	PGMATRIX_MULTIPLY_COPY(pMatDummy, pMatB, pMatC);	
	PGMATRIX_MULTIPLY_ADD(pMatResult, pMatA, pMatDummy);
}
#endif

#if GMATRIX_DEFINE_TRIPLEMULTIPLY_ADD_EXTENDED

void PGMATRIX_TRIPLEMULTIPLY_ADD_EXTENDED(PGMATRIX pMatResult, PGMATRIX pMatA, int FlagTransposeA, PGMATRIX pMatB, int FlagTransposeB, PGMATRIX pMatC, int FlagTransposeC, PGMATRIX pMatDummy) 

{

//	GMATRIX_ASSERT("PGMATRIX_TRIPLEMULTIPLY_ADD_EXTENDED",pMatA->Nc != pMatB->Nr);	

//	GMATRIX_ASSERT("PGMATRIX_TRIPLEMULTIPLY_ADD_EXTENDED",pMatB->Nc != pMatC->Nr);	

//	GMATRIX_ASSERT("PGMATRIX_TRIPLEMULTIPLY_ADD_EXTENDED",pMatResult->Nr != pMatA->Nr); 

//	GMATRIX_ASSERT("PGMATRIX_TRIPLEMULTIPLY_ADD_EXTENDED",pMatResult->Nc != pMatC->Nc); 

	PGMATRIX_MULTIPLY_COPY_EXTENDED(pMatDummy, pMatB, FlagTransposeB, pMatC, FlagTransposeC);	

	PGMATRIX_MULTIPLY_ADD_EXTENDED(pMatResult, pMatA, FlagTransposeA, pMatDummy, FALSE);

}

#endif

/**********************************************************************************************
***** GMatrix: Submatrices
**********************************************************************************************/
#if GMATRIX_DEFINE_SUBMATRIX_COPY
void PGMATRIX_SUBMATRIX_COPY(PGMATRIX pMat, int nl, int nc, PGMATRIX pMatOrigin)
{
	int i,j;
	GMATRIX_ASSERT("PGMATRIX_SUBMATRIX_COPY",nl<1);	
	GMATRIX_ASSERT("PGMATRIX_SUBMATRIX_COPY",nc<1);	
	GMATRIX_ASSERT("PGMATRIX_SUBMATRIX_COPY",pMat->Nr < (nl+pMatOrigin->Nr-1));	
	GMATRIX_ASSERT("PGMATRIX_SUBMATRIX_COPY",pMat->Nc < (nc+pMatOrigin->Nc-1));	
    for(i=nl;(i<=(nl+pMatOrigin->Nr-1)) && (i<=pMat->Nr);++i){ 
		for(j=nc;(j<=(nc+pMatOrigin->Nc-1)) && (j<=pMat->Nc);++j){ 
			PGMATRIX_DATA(pMat,i,j) = PGMATRIX_DATA(pMatOrigin,i-nl+1,j-nc+1); 
		} 
    }
}
#endif

#if GMATRIX_DEFINE_SUBMATRIX_ADD
void PGMATRIX_SUBMATRIX_ADD(PGMATRIX pMat, int nl, int nc, PGMATRIX pMatOrigin)
{
	int i,j;
	GMATRIX_ASSERT("PGMATRIX_SUBMATRIX_ADD",nl<1);	
	GMATRIX_ASSERT("PGMATRIX_SUBMATRIX_ADD",nc<1);	
	GMATRIX_ASSERT("PGMATRIX_SUBMATRIX_ADD",pMat->Nr < (nl+pMatOrigin->Nr-1));	
	GMATRIX_ASSERT("PGMATRIX_SUBMATRIX_ADD",pMat->Nc < (nc+pMatOrigin->Nc-1));	
    for(i=nl;(i<=(nl+pMatOrigin->Nr-1)) && (i<=pMat->Nr);++i){ 
		for(j=nc;(j<=(nc+pMatOrigin->Nc-1)) && (j<=pMat->Nc);++j){ 
			PGMATRIX_DATA(pMat,i,j) += PGMATRIX_DATA(pMatOrigin,i-nl+1,j-nc+1); 
		} 
    }
}
#endif

#if GMATRIX_DEFINE_SUBMATRIX_COPY_EXTENDED
void PGMATRIX_SUBMATRIX_COPY_EXTENDED(PGMATRIX pMat, int nl, int nc, PGMATRIX pMatOrigin, int FlagTransposeOrigin)
{
	int i,j;
	GMATRIX_ASSERT("PGMATRIX_SUBMATRIX_COPY_EXTENDED",nl<1);	
	GMATRIX_ASSERT("PGMATRIX_SUBMATRIX_COPY_EXTENDED",nc<1);
	if (!FlagTransposeOrigin){
		GMATRIX_ASSERT("PGMATRIX_SUBMATRIX_COPY_EXTENDED (pMatOrigin is not transposed)",pMat->Nr < (nl+pMatOrigin->Nr-1));	
		GMATRIX_ASSERT("PGMATRIX_SUBMATRIX_COPY_EXTENDED (pMatOrigin is not transposed)",pMat->Nc < (nc+pMatOrigin->Nc-1));	
	    for(i=nl;(i<=(nl+pMatOrigin->Nr-1)) && (i<=pMat->Nr);++i){ 
			for(j=nc;(j<=(nc+pMatOrigin->Nc-1)) && (j<=pMat->Nc);++j){ 
				PGMATRIX_DATA(pMat,i,j) = PGMATRIX_DATA(pMatOrigin,i-nl+1,j-nc+1); 
			} 
		}
	}
	else{
		GMATRIX_ASSERT("PGMATRIX_SUBMATRIX_COPY_EXTENDED (pMatOrigin is transposed)",pMat->Nr < (nl+pMatOrigin->Nc-1));	
		GMATRIX_ASSERT("PGMATRIX_SUBMATRIX_COPY_EXTENDED (pMatOrigin is transposed)",pMat->Nc < (nc+pMatOrigin->Nr-1));	
	    for(i=nl;(i<=(nl+pMatOrigin->Nc-1)) && (i<=pMat->Nr);++i){ 
			for(j=nc;(j<=(nc+pMatOrigin->Nr-1)) && (j<=pMat->Nc);++j){ 
				PGMATRIX_DATA(pMat,i,j) = PGMATRIX_DATA(pMatOrigin,j-nc+1,i-nl+1); 
			} 
		}
	}
}
#endif

#if GMATRIX_DEFINE_SUBMATRIX_FILL
void PGMATRIX_SUBMATRIX_FILL(PGMATRIX pMat,int nlb,int nle,int ncb,int nce,GMATRIX_FLOAT Value)
{
	int i,j;
	GMATRIX_ASSERT("PGMATRIX_SUBMATRIX_FILL",nlb<1);	
	GMATRIX_ASSERT("PGMATRIX_SUBMATRIX_FILL",ncb<1);	
	GMATRIX_ASSERT("PGMATRIX_SUBMATRIX_FILL",pMat->Nr < nle);	
	GMATRIX_ASSERT("PGMATRIX_SUBMATRIX_FILL",pMat->Nc < nce);	
    for(i=nlb;(i<=nle) && (i<=pMat->Nr);++i){ 
		for(j=ncb;(j<=nce) && (j<=pMat->Nc);++j){ 
			PGMATRIX_DATA(pMat,i,j) = Value; 
		} 
    }
}
#endif

#if GMATRIX_DEFINE_SUBMATRIX_MULTIPLY_CONST
void PGMATRIX_SUBMATRIX_MULTIPLY_CONST(PGMATRIX pMat,int nlb,int nle,int ncb,int nce,GMATRIX_FLOAT Value)
{
	int i,j;
	GMATRIX_ASSERT("PGMATRIX_SUBMATRIX_MULTIPLY_CONST",nlb<1);	
	GMATRIX_ASSERT("PGMATRIX_SUBMATRIX_MULTIPLY_CONST",ncb<1);	
	GMATRIX_ASSERT("PGMATRIX_SUBMATRIX_MULTIPLY_CONST",pMat->Nr < nle);	
	GMATRIX_ASSERT("PGMATRIX_SUBMATRIX_MULTIPLY_CONST",pMat->Nc < nce);	
    for(i=nlb;(i<=nle) && (i<=pMat->Nr);++i){ 
		for(j=ncb;(j<=nce) && (j<=pMat->Nc);++j){ 
			PGMATRIX_DATA(pMat,i,j) *= Value; 
		} 
    }
}
#endif


