/**********************************************************************************************
***********************************************************************************************
***  File:			GMatrix_linalg.h
***	 Author:		Geovany Araujo Borges
***	 Contents:		GMatrix_linalg header file.
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

#ifndef GMATRIX_LINALG_H
#define	GMATRIX_LINALG_H

/**********************************************************************************************
***** GMatrix: Function prototypes
**********************************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

#define GMATRIX_LUDCMP(MatLU, Mat) PGMATRIX_LUDCMP(&MatLU, &Mat)
GMATRIX_FLOAT PGMATRIX_LUDCMP(PGMATRIX pMatLU, PGMATRIX pMat);

#define GMATRIX_GAUSSJORDAN(AInv, X, A, B) PGMATRIX_GAUSSJORDAN(&AInv, &X, &A, &B)
void PGMATRIX_GAUSSJORDAN(PGMATRIX pAInverse, PGMATRIX pX, PGMATRIX pA, PGMATRIX pB);

#define GMATRIX_CHOLESKY(L, A) PGMATRIX_CHOLESKY(&L, &A)
void PGMATRIX_CHOLESKY(PGMATRIX pL, PGMATRIX pA);

#define GMATRIX_DETERMINANT2(Mat) PGMATRIX_DETERMINANT2(&Mat)
GMATRIX_FLOAT PGMATRIX_DETERMINANT2(PGMATRIX pMat);

#define GMATRIX_DETERMINANT3(Mat) PGMATRIX_DETERMINANT3(&Mat)
GMATRIX_FLOAT PGMATRIX_DETERMINANT3(PGMATRIX pMat);

#define GMATRIX_DETERMINANT(Mat, MatDummy) PGMATRIX_DETERMINANT(&Mat, &MatDummy)
GMATRIX_FLOAT PGMATRIX_DETERMINANT(PGMATRIX pMat, PGMATRIX pMatDummy);

#define GMATRIX_INVERSE2_COPY(MatInverse, Mat) PGMATRIX_INVERSE2_COPY(&MatInverse, &Mat)
void PGMATRIX_INVERSE2_COPY(PGMATRIX pMatInverse, PGMATRIX pMat); 

#define GMATRIX_INVERSE3_COPY(MatInverse, Mat) PGMATRIX_INVERSE3_COPY(&MatInverse, &Mat)
void PGMATRIX_INVERSE3_COPY(PGMATRIX pMatInverse, PGMATRIX pMat); 

#define	GMATRIX_INVERSE_COPY(MatInverse, Mat) PGMATRIX_INVERSE_COPY(&MatInverse, &Mat)
void PGMATRIX_INVERSE_COPY(PGMATRIX pMatInverse, PGMATRIX pMat); 

#define	GMATRIX_INVERSE(Mat) PGMATRIX_INVERSE(&Mat)
void PGMATRIX_INVERSE(PGMATRIX pMat); 

#define	GMATRIX_SVD(U,S,V,Mat,FlagSorted) PGMATRIX_SVD(&U,&S,&V,&Mat,FlagSorted)
void PGMATRIX_SVD(PGMATRIX pU,PGMATRIX pS,PGMATRIX pV,PGMATRIX pMat, unsigned char FlagSorted);

#define	GMATRIX_PSEUDOINVERSE(Apinv,A,MatDummy) PGMATRIX_PSEUDOINVERSE(&Apinv,&A,&MatDummy)
void PGMATRIX_PSEUDOINVERSE(PGMATRIX pApinv, PGMATRIX pA, PGMATRIX pMatDummy);

#define GMATRIX_LEFT_PSEUDOINVERSE_COPY(Apinv,A,MatDummy) PGMATRIX_LEFT_PSEUDOINVERSE_COPY(&Apinv,&A,&MatDummy)
void PGMATRIX_LEFT_PSEUDOINVERSE_COPY(PGMATRIX pApinv, PGMATRIX pA, PGMATRIX pMatDummy);

#define GMATRIX_DERIVATIVE_LEFT_PSEUDOINVERSE(dApinvdx,dAdx,A,Apinv,MatDummy1,MatDummy2) PGMATRIX_DERIVATIVE_LEFT_PSEUDOINVERSE(&dApinvdx,&dAdx,&A,&Apinv,&MatDummy1,&MatDummy2)
void PGMATRIX_DERIVATIVE_LEFT_PSEUDOINVERSE(PGMATRIX pdApinvdx, PGMATRIX pdAdx, PGMATRIX pA, PGMATRIX pApinv, PGMATRIX pMatDummy1, PGMATRIX pMatDummy2);

#define GMATRIX_RANK_FROMSVD(U,S,V) PGMATRIX_RANK_FROMSVD(&U,&S,&V)
int PGMATRIX_RANK_FROMSVD(PGMATRIX pU, PGMATRIX pS, PGMATRIX pV);

#define GMATRIX_RANK(Mat) PGMATRIX_RANK(&Mat)
int PGMATRIX_RANK(PGMATRIX pMat);

#ifdef __cplusplus
}
#endif

#endif // GMATRIX_LINALG_H
