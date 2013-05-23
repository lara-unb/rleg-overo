/**********************************************************************************************
***********************************************************************************************
***  File:			GMatrix.h
***	 Author:		Geovany Araujo Borges
***	 Contents:		GMatrix header file.
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

#ifndef GMATRIX_H
#define	GMATRIX_H

/**********************************************************************************************
***** GMatrix: External defines used to guide library use.
**********************************************************************************************/
//#define GMATRIX_CMEX // Uncomment this line if CMEX is used.

//#define GMATRIX_FLOAT		float
#define GMATRIX_FLOAT		double

// Define the default print command:
#ifndef GMATRIX_PRINTCOMMAND
#define	GMATRIX_PRINTCOMMAND printf
#endif

// Define the default abort command:
#ifndef GMATRIX_ABORTCOMMAND
#define	GMATRIX_ABORTCOMMAND getchar();exit(1);
#endif

// Define BOOL:
#ifndef BOOL
#define	BOOL unsigned char
#define	TRUE 1
#define	FALSE 0
#endif


/**********************************************************************************************
***** GMatrix: Defines will be used in the project. All of them can be defined at compilation 
***** time according to compiler options.
**********************************************************************************************/
#if !defined(GMATRIX_DEFINE_ALL)
	#define GMATRIX_DEFINE_ALL 								(1) 
#endif
#if !defined(GMATRIX_DEFINE_ALLOC)
	#define GMATRIX_DEFINE_ALLOC 							(0 + GMATRIX_DEFINE_ALL) 
#endif
#if !defined(GMATRIX_DEFINE_SETSIZE)					
	#define  GMATRIX_DEFINE_SETSIZE							(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_PRINT)					
	#define  GMATRIX_DEFINE_PRINT							(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_PRINT_MATLABFORM)	
	#define  GMATRIX_DEFINE_PRINT_MATLABFORM				(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_PRINTROW)	
	#define  GMATRIX_DEFINE_PRINTROW						(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_PRINT_EXP)						
	#define  GMATRIX_DEFINE_PRINT_EXP						(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_INFO)						
	#define  GMATRIX_DEFINE_INFO							(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_ZEROES)				
	#define  GMATRIX_DEFINE_ZEROES							(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_SUMLINCOL)		
	#define  GMATRIX_DEFINE_SUMLINCOL						(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_ONES)		
	#define  GMATRIX_DEFINE_ONES							(0 + GMATRIX_DEFINE_ALL)	
#endif
#if !defined(GMATRIX_DEFINE_RAND)							
	#define  GMATRIX_DEFINE_RAND							(0 + GMATRIX_DEFINE_ALL)				
#endif
#if !defined(GMATRIX_DEFINE_RANDN)					
	#define  GMATRIX_DEFINE_RANDN							(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_IDENTITY)			
	#define  GMATRIX_DEFINE_IDENTITY						(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_WILKINSON)	
	#define  GMATRIX_DEFINE_WILKINSON						(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_ORTHOGONAL)						
	#define  GMATRIX_DEFINE_ORTHOGONAL						(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_COPY)						
	#define  GMATRIX_DEFINE_COPY							(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_COPY_COLUMN)			
	#define  GMATRIX_DEFINE_COPY_COLUMN						(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_COPY_ROW)		
	#define  GMATRIX_DEFINE_COPY_ROW						(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_TRANSPOSE_COPY)					
	#define  GMATRIX_DEFINE_TRANSPOSE_COPY					(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_TRACE)						
	#define  GMATRIX_DEFINE_TRACE							(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_SUMENTRIES)				
	#define  GMATRIX_DEFINE_SUMENTRIES						(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_SUMABSOLUTEENTRIES)	
	#define  GMATRIX_DEFINE_SUMABSOLUTEENTRIES				(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_LUDCMP)			
	#define  GMATRIX_DEFINE_LUDCMP							(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_GAUSSJORDAN)
	#define  GMATRIX_DEFINE_GAUSSJORDAN						(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_CHOLESKY)
	#define  GMATRIX_DEFINE_CHOLESKY						(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_DETERMINANT2)					
	#define  GMATRIX_DEFINE_DETERMINANT2					(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_DETERMINANT3)			
	#define  GMATRIX_DEFINE_DETERMINANT3					(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_DETERMINANT)		
	#define  GMATRIX_DEFINE_DETERMINANT						(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_INVERSE2_COPY)					
	#define  GMATRIX_DEFINE_INVERSE2_COPY					(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_INVERSE3_COPY)				
	#define  GMATRIX_DEFINE_INVERSE3_COPY					(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_INVERSE_COPY)			
	#define  GMATRIX_DEFINE_INVERSE_COPY					(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_INVERSE)			
	#define  GMATRIX_DEFINE_INVERSE							(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_SWAP_ROW)	
	#define  GMATRIX_DEFINE_SWAP_ROW						(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_SWAP_COLUMN)					
	#define  GMATRIX_DEFINE_SWAP_COLUMN						(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_SVD)							
	#define  GMATRIX_DEFINE_SVD								(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_ADD_CONST)				
	#define  GMATRIX_DEFINE_ADD_CONST						(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_CROSS)		
	#define  GMATRIX_DEFINE_CROSS							(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_MULTIPLY_CONST)		
	#define  GMATRIX_DEFINE_MULTIPLY_CONST					(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_MULTIPLY_CONST_COPY)				
	#define  GMATRIX_DEFINE_MULTIPLY_CONST_COPY				(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_MULTIPLY_CONST_ADD)			
	#define  GMATRIX_DEFINE_MULTIPLY_CONST_ADD				(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_ADD_COPY)				
	#define  GMATRIX_DEFINE_ADD_COPY						(0 + GMATRIX_DEFINE_ALL)	
#endif
#if !defined(GMATRIX_DEFINE_ADD)				
	#define  GMATRIX_DEFINE_ADD								(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_SUBSTRACT_COPY)					
	#define  GMATRIX_DEFINE_SUBSTRACT_COPY					(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_SUBSTRACT_IDENTITY_COPY)		
	#define  GMATRIX_DEFINE_SUBSTRACT_IDENTITY_COPY			(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_SUBSTRACT_IDENTITY)		
	#define  GMATRIX_DEFINE_SUBSTRACT_IDENTITY				(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_SUBSTRACT)			
	#define  GMATRIX_DEFINE_SUBSTRACT						(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_MULTIPLY_COPY)	
	#define  GMATRIX_DEFINE_MULTIPLY_COPY					(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_MULTIPLY_COPY_EXTENDED)			
	#define  GMATRIX_DEFINE_MULTIPLY_COPY_EXTENDED			(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_MULTIPLY_ADD)				
	#define  GMATRIX_DEFINE_MULTIPLY_ADD					(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_MULTIPLY_ADD_EXTENDED)				
	#define  GMATRIX_DEFINE_MULTIPLY_ADD_EXTENDED				(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_MULTIPLY_SUBSTRACT)	
	#define  GMATRIX_DEFINE_MULTIPLY_SUBSTRACT				(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_TRIPLEMULTIPLY_COPY)				
	#define  GMATRIX_DEFINE_TRIPLEMULTIPLY_COPY				(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_TRIPLEMULTIPLY_COPY_EXTENDED)
	#define  GMATRIX_DEFINE_TRIPLEMULTIPLY_COPY_EXTENDED	(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_TRIPLEMULTIPLY_ADD)	
	#define  GMATRIX_DEFINE_TRIPLEMULTIPLY_ADD				(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_TRIPLEMULTIPLY_ADD_EXTENDED)	
	#define  GMATRIX_DEFINE_TRIPLEMULTIPLY_ADD_EXTENDED			(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_COVARIANCE_PROPAGATION_COPY)		
	#define  GMATRIX_DEFINE_COVARIANCE_PROPAGATION_COPY		(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_COVARIANCE_PROPAGATION_ADD)	
	#define  GMATRIX_DEFINE_COVARIANCE_PROPAGATION_ADD		(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_MAHALANOBIS_DISTANCE)	
	#define  GMATRIX_DEFINE_MAHALANOBIS_DISTANCE			(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_SUBMATRIX_COPY)	
	#define  GMATRIX_DEFINE_SUBMATRIX_COPY					(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_SUBMATRIX_ADD)					
	#define  GMATRIX_DEFINE_SUBMATRIX_ADD					(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_SUBMATRIX_COPY_EXTENDED)		
	#define  GMATRIX_DEFINE_SUBMATRIX_COPY_EXTENDED			(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_SUBMATRIX_FILL)			
	#define  GMATRIX_DEFINE_SUBMATRIX_FILL					(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_SUBMATRIX_MULTIPLY_CONST)		
	#define  GMATRIX_DEFINE_SUBMATRIX_MULTIPLY_CONST		(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_PSEUDOINVERSE)			
	#define  GMATRIX_DEFINE_PSEUDOINVERSE					(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_LEFT_PSEUDOINVERSE_COPY)			
	#define  GMATRIX_DEFINE_LEFT_PSEUDOINVERSE_COPY			(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_DERIVATIVE_LEFT_PSEUDOINVERSE)	
	#define  GMATRIX_DEFINE_DERIVATIVE_LEFT_PSEUDOINVERSE	(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_RANK_FROMSVD)				
	#define  GMATRIX_DEFINE_RANK_FROMSVD					(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_RANK)				
	#define  GMATRIX_DEFINE_RANK							(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_MEAN)		
	#define  GMATRIX_DEFINE_MEAN							(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_VARIANCE)						
	#define  GMATRIX_DEFINE_VARIANCE						(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(GMATRIX_DEFINE_NORM)	
	#define  GMATRIX_DEFINE_NORM							(0 + GMATRIX_DEFINE_ALL)
#endif
#if !defined(MATRIX_DEFINE_SPARSITY_COEFFICIENT)	
	#define  GMATRIX_DEFINE_SPARSITY_COEFFICIENT			(0 + GMATRIX_DEFINE_ALL)
#endif

/**********************************************************************************************
***** GMatrix: Type Definitions
**********************************************************************************************/
/*! 
 *  GMATRIX is the data structure of a matrix or a vector. PGMATRIX is a pointer for GMATRIX.
 */

// Main GMATRIX type structure and pointer
typedef struct{	
		int Nr;	/* Number of rows of the matrix */
		int Nc;	/* Number of columns of the matrix */
		int MaxSize; /* Number of entries of *Data. Used mainly in matrix resizing of dummy matrices */
		GMATRIX_FLOAT *Data; /* Pointer to entries of GMATRIX. The entries are organized sequentially, column-by-column */	
} GMATRIX, *PGMATRIX;

/**********************************************************************************************
***** GMatrix: Internal macro operators and constants:
**********************************************************************************************/
#define GMATRIXCONST_PI				3.14159265358979310000
#define GMATRIXCONST_2PI			6.28318530717958620000
#define GMATRIXCONST_E	  			2.71828182845904550000

#define GMATRIXMACRO_SIGN(a)		((a) >= 0.0 ? 1.0 : -1.0)
#define GMATRIXMACRO_SWAP(a,b)		{temp = (a); (a) = (b); (b) = temp;}
#define GMATRIXMACRO_MAX(a,b)		((a) > (b) ? (a) : (b))
#define GMATRIXMACRO_MIN(a,b)		((a) > (b) ? (b) : (a))
#define GMATRIXMACRO_SQR(a)			((a)*(a))
#define GMATRIXMACRO_SIGNCHAR(a)	((a) >= 0.0 ? ' ' : '-')

/**********************************************************************************************
***** GMatrix: Type Definitions
**********************************************************************************************/
/* 
 *  DUMMY_MATRICES is a structure containig four previously allocated matrices. 
 *
 *  This structure is used by some functions of GMatrix Library. Since its contents is previously allocated, 
 *  there is no need of dynamic memory allocation, which may critical in real-time threads. 
 *  PDUMMY_MATRICES is a pointer for DUMMY_MATRICES.
 */

// Dummy matrices:
typedef struct{
	PGMATRIX pMat1; /* Auxiliary Matrix 1 */
	PGMATRIX pMat2; /* Auxiliary Matrix 2 */
	PGMATRIX pMat3; /* Auxiliary Matrix 3 */
	PGMATRIX pMat4; /* Auxiliary Matrix 4 */
} DUMMY_MATRICES,*PDUMMY_MATRICES;

/**********************************************************************************************
***** GMatrix: Declaration
**********************************************************************************************/
#define GMATRIX_DECLARE(MatName, NLIN, NCOL) \
	GMATRIX_FLOAT MatName##_Data[NLIN*NCOL]; \
	GMATRIX MatName = {NLIN,NCOL,NLIN*NCOL,&MatName##_Data[0]}

/**********************************************************************************************
***** GMatrix: Data Access
**********************************************************************************************/
#define  GMATRIX_DATA(Mat,i,j)  (*(&Mat.Data[i-1+(j-1)*Mat.Nr]))
#define PGMATRIX_DATA(pMat,i,j) (*(&pMat->Data[i-1+(j-1)*pMat->Nr]))

/**********************************************************************************************
***** GMatrix: Error Handling
**********************************************************************************************/
#define GMATRIX_ERROR(msg) {GMATRIX_PRINTCOMMAND("\n%s",msg); GMATRIX_ABORT_FUNCTION;}

#define GMATRIX_ASSERT(CallingFunction,TestFailCode)\
	if(TestFailCode){ \
		GMATRIX_PRINTCOMMAND("\n*** Warning ( %s ) in function %s",#TestFailCode,CallingFunction);\
		GMATRIX_ABORT_FUNCTION;\
	}

// Defines related to the use in CMEX environment:
#ifndef	GMATRIX_CMEX
	/*! 
	 *  GMATRIX_MATLAB_DATAHEAD is a structure used for saving/loading matrices in MATLAB v4.0 format..
	 */
	// Datahead for MATLAB 4.0 data interfacing:
	typedef struct{
			long type; /* Data type */
			long mrows; /* Number of rows */
			long ncols; /* Number of columns */
			long imagf; /* Imaginary flag */
			long namlen; /* Length of variable name  */
	} GMATRIX_MATLAB_DATAHEAD;
#endif

// Define for default abort function:
#ifndef GMATRIX_ABORT_FUNCTION
	#ifndef GMATRIX_CMEX
		#define GMATRIX_ABORT_FUNCTION {GMATRIX_PRINTCOMMAND("\nAborting...");GMATRIX_ABORTCOMMAND;}
	#else
		#define GMATRIX_ABORT_FUNCTION mexErrMsgTxt("Aborting...")
	#endif
#endif

/**********************************************************************************************
***** GMatrix: Function prototypes
**********************************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif


PGMATRIX PGMATRIX_ALLOC(int Nr, int Nc);
void PGMATRIX_FREE(PGMATRIX pMatrix);

#define GMATRIX_SETSIZE(Mat,Nll,Ncc) PGMATRIX_SETSIZE(&Mat,Nll,Ncc)
void PGMATRIX_SETSIZE(PGMATRIX pMat,int Nll, int Ncc);

#define GMATRIX_PRINT(Mat) PGMATRIX_PRINT_NAMED(#Mat,&Mat)
#define PGMATRIX_PRINT(pMat) PGMATRIX_PRINT_NAMED(#pMat,pMat)

#define GMATRIX_PRINT_MATLABFORM(Mat) PGMATRIX_PRINT_MATLABFORM_NAMED(#Mat,&Mat)
#define PGMATRIX_PRINT_MATLABFORM(pMat) PGMATRIX_PRINT_MATLABFORM_NAMED(#pMat,pMat)

#define GMATRIX_PRINTROW(Mat,i) PGMATRIX_PRINTROW_NAMED(#Mat,&Mat,i)
#define PGMATRIX_PRINTROW(pMat,i) PGMATRIX_PRINTROW_NAMED(#pMat,pMat,i)

#define GMATRIX_PRINT_EXP(Mat) PGMATRIX_PRINT_EXP_NAMED(#Mat,&Mat)
#define PGMATRIX_PRINT_EXP(pMat) PGMATRIX_PRINT_EXP_NAMED(#pMat,pMat)

void PGMATRIX_PRINT_NAMED(char* NameString,PGMATRIX pMat);
void PGMATRIX_PRINT_MATLABFORM_NAMED(char* NameString,PGMATRIX pMat);
void PGMATRIX_PRINTROW_NAMED(char* NameString,PGMATRIX pMat,int i);
void PGMATRIX_PRINT_EXP_NAMED(char* NameString,PGMATRIX pMat);

#define GMATRIX_INFO(Mat) PGMATRIX_INFO_NAMED(#Mat,&Mat)
#define PGMATRIX_INFO(pMat) PGMATRIX_INFO_NAMED(#pMat,pMat)

void PGMATRIX_INFO_NAMED(char* NameString,PGMATRIX pMat);

#define GMATRIX_ZEROES(Mat) PGMATRIX_ZEROES(&Mat)
void PGMATRIX_ZEROES(PGMATRIX pMat);

#define GMATRIX_SUMLINCOL(Mat) PGMATRIX_SUMLINCOL(&Mat)
void PGMATRIX_SUMLINCOL(PGMATRIX pMat);

#define GMATRIX_ONES(Mat) PGMATRIX_ONES(&Mat)
void PGMATRIX_ONES(PGMATRIX pMat);

#define GMATRIX_RAND(Mat) PGMATRIX_RAND(&Mat) 
void PGMATRIX_RAND(PGMATRIX pMat);

#define GMATRIX_RANDN(Mat) PGMATRIX_RANDN(&Mat)
void PGMATRIX_RANDN(PGMATRIX pMat);

#define GMATRIX_IDENTITY(Mat) PGMATRIX_IDENTITY(&Mat) 
void PGMATRIX_IDENTITY(PGMATRIX pMat); 

#define GMATRIX_WILKINSON(Mat) PGMATRIX_WILKINSON(&Mat)
void PGMATRIX_WILKINSON(PGMATRIX pMat);

#define GMATRIX_ORTHOGONAL(Mat,Type) PGMATRIX_ORTHOGONAL(&Mat,Type)
void PGMATRIX_ORTHOGONAL(PGMATRIX pMat,int Type);

#define GMATRIX_COPY(MatResult, Mat) PGMATRIX_COPY(&MatResult, &Mat)
void PGMATRIX_COPY(PGMATRIX pMatResult, PGMATRIX pMat);

#define GMATRIX_COPY_COLUMN(MatResult, ColDest, Mat, ColOrigin) PGMATRIX_COPY_COLUMN(&MatResult, ColDest, &Mat, ColOrigin)
void PGMATRIX_COPY_COLUMN(PGMATRIX pMatResult, int ColDest, PGMATRIX pMat, int ColOrigin);

#define GMATRIX_COPY_ROW(MatResult, RowDest, Mat, RowOrigin) PGMATRIX_COPY_ROW(&MatResult, RowDest, &Mat, RowOrigin)
void PGMATRIX_COPY_ROW(PGMATRIX pMatResult, int RowDest, PGMATRIX pMat, int RowOrigin);

#define GMATRIX_TRANSPOSE_COPY(MatTranspose, Mat) PGMATRIX_TRANSPOSE_COPY(&MatTranspose, &Mat)
void PGMATRIX_TRANSPOSE_COPY(PGMATRIX pMatTranspose, PGMATRIX pMat); 

#define GMATRIX_TRACE(Mat) PGMATRIX_TRACE(&Mat)
GMATRIX_FLOAT PGMATRIX_TRACE(PGMATRIX pMat);

#define GMATRIX_SUMENTRIES(Mat) PGMATRIX_SUMENTRIES(&Mat)
GMATRIX_FLOAT PGMATRIX_SUMENTRIES(PGMATRIX pMat);

#define GMATRIX_SUMABSOLUTEENTRIES(Mat) PGMATRIX_SUMABSOLUTEENTRIES(&Mat)
GMATRIX_FLOAT PGMATRIX_SUMABSOLUTEENTRIES(PGMATRIX pMat);

#define GMATRIX_SWAP_ROW(Mat,i,j)	PGMATRIX_SWAP_ROW(&Mat,i,j)
void PGMATRIX_SWAP_ROW(PGMATRIX pMat,int i, int j);

#define GMATRIX_SWAP_COLUMN(Mat,i,j)	PGMATRIX_SWAP_COLUMN(&Mat,i,j)
void PGMATRIX_SWAP_COLUMN(PGMATRIX pMat,int i, int j);

#define GMATRIX_ADD_CONST(Mat,Value) PGMATRIX_ADD_CONST(&Mat,Value)
void PGMATRIX_ADD_CONST(PGMATRIX pMat,GMATRIX_FLOAT Value);

#define GMATRIX_CROSS_COPY(MatResult, MatA, MatB) PGMATRIX_CROSS_COPY(&MatResult, &MatA, &MatB)

void PGMATRIX_CROSS_COPY(PGMATRIX pMatResult, PGMATRIX pMatA, PGMATRIX pMatB) ;

#define GMATRIX_MULTIPLY_CONST(Mat,Value) PGMATRIX_MULTIPLY_CONST(&Mat,Value)
void PGMATRIX_MULTIPLY_CONST(PGMATRIX pMat,GMATRIX_FLOAT Value);

#define GMATRIX_MULTIPLY_CONST_COPY(MatResult, Mat,Value) PGMATRIX_MULTIPLY_CONST_COPY(&MatResult, &Mat,Value)
void PGMATRIX_MULTIPLY_CONST_COPY(PGMATRIX pMatResult, PGMATRIX pMat,GMATRIX_FLOAT Value);

#define GMATRIX_MULTIPLY_CONST_ADD(MatResult, Mat,Value) PGMATRIX_MULTIPLY_CONST_ADD(&MatResult, &Mat,Value)
void PGMATRIX_MULTIPLY_CONST_ADD(PGMATRIX pMatResult, PGMATRIX pMat,GMATRIX_FLOAT Value);

#define GMATRIX_NORM(Mat) PGMATRIX_NORM(&Mat)
GMATRIX_FLOAT PGMATRIX_NORM(PGMATRIX pMat);

#define GMATRIX_ADD_COPY(MatResult, MatA, MatB) PGMATRIX_ADD_COPY(&MatResult, &MatA, &MatB)
void PGMATRIX_ADD_COPY(PGMATRIX pMatResult, PGMATRIX pMatA, PGMATRIX pMatB);

#define GMATRIX_ADD(MatA, MatB) PGMATRIX_ADD(&MatA, &MatB)
void PGMATRIX_ADD(PGMATRIX pMatA, PGMATRIX pMatB);

#define GMATRIX_SUBTRACT_COPY(MatResult, MatA, MatB) PGMATRIX_SUBTRACT_COPY(&MatResult, &MatA, &MatB)
#define PGMATRIX_SUBTRACT_COPY(pMatResult, pMatA, pMatB) PGMATRIX_SUBSTRACT_COPY(pMatResult, pMatA, pMatB)
#define GMATRIX_SUBSTRACT_COPY(MatResult, MatA, MatB) PGMATRIX_SUBSTRACT_COPY(&MatResult, &MatA, &MatB)
void PGMATRIX_SUBSTRACT_COPY(PGMATRIX pMatResult, PGMATRIX pMatA, PGMATRIX pMatB);

#define GMATRIX_SUBSTRACT_IDENTITY_COPY(MatResult, Mat) PGMATRIX_SUBSTRACT_IDENTITY_COPY(&MatResult, &Mat)
void PGMATRIX_SUBSTRACT_IDENTITY_COPY(PGMATRIX pMatResult, PGMATRIX pMat);

#define GMATRIX_SUBSTRACT_IDENTITY(Mat) PGMATRIX_SUBSTRACT_IDENTITY(&Mat)
void PGMATRIX_SUBSTRACT_IDENTITY(PGMATRIX pMat);

#define GMATRIX_SUBSTRACT(MatA, MatB) PGMATRIX_SUBSTRACT(&MatA, &MatB)
void PGMATRIX_SUBSTRACT(PGMATRIX pMatA, PGMATRIX pMatB);

#define GMATRIX_MULTIPLY_COPY(MatResult, MatA, MatB) PGMATRIX_MULTIPLY_COPY(&MatResult, &MatA, &MatB)
void PGMATRIX_MULTIPLY_COPY(PGMATRIX pMatResult, PGMATRIX pMatA, PGMATRIX pMatB);

#define GMATRIX_MULTIPLY_COPY_EXTENDED(MatResult, MatA, FlagTransposeA, MatB, FlagTransposeB) PGMATRIX_MULTIPLY_COPY_EXTENDED(&MatResult, &MatA, FlagTransposeA, &MatB, FlagTransposeB)
void PGMATRIX_MULTIPLY_COPY_EXTENDED(PGMATRIX pMatResult, PGMATRIX pMatA, int FlagTransposeA, PGMATRIX pMatB, int FlagTransposeB) ;

#define GMATRIX_MULTIPLY_ADD(MatResult, MatA, MatB) PGMATRIX_MULTIPLY_ADD(&MatResult, &MatA, &MatB)
void PGMATRIX_MULTIPLY_ADD(PGMATRIX pMatResult, PGMATRIX pMatA, PGMATRIX pMatB) ;

#define GMATRIX_MULTIPLY_ADD_EXTENDED(MatResult, MatA, FlagTransposeA, MatB, FlagTransposeB) PGMATRIX_MULTIPLY_ADD_EXTENDED(&MatResult, &MatA, FlagTransposeA, &MatB, FlagTransposeB)

void PGMATRIX_MULTIPLY_ADD_EXTENDED(PGMATRIX pMatResult, PGMATRIX pMatA, int FlagTransposeA, PGMATRIX pMatB, int FlagTransposeB) ;

#define GMATRIX_MULTIPLY_SUBSTRACT(MatResult, MatA, MatB) PGMATRIX_MULTIPLY_SUBSTRACT(&MatResult, &MatA, &MatB)
void PGMATRIX_MULTIPLY_SUBSTRACT(PGMATRIX pMatResult, PGMATRIX pMatA, PGMATRIX pMatB) ;

#define GMATRIX_TRIPLEMULTIPLY_COPY(MatResult, MatA, MatB, MatC, MatDummy) PGMATRIX_TRIPLEMULTIPLY_COPY(&MatResult, &MatA, &MatB, &MatC, &MatDummy)
void PGMATRIX_TRIPLEMULTIPLY_COPY(PGMATRIX pMatResult, PGMATRIX pMatA, PGMATRIX pMatB, PGMATRIX pMatC, PGMATRIX pMatDummy) ;

#define GMATRIX_TRIPLEMULTIPLY_COPY_EXTENDED(MatResult, MatA, FlagTransposeA, MatB, FlagTransposeB, MatC, FlagTransposeC, MatDummy) PGMATRIX_TRIPLEMULTIPLY_COPY_EXTENDED(&MatResult, &MatA, FlagTransposeA, &MatB, FlagTransposeB, &MatC, FlagTransposeC, &MatDummy)
void PGMATRIX_TRIPLEMULTIPLY_COPY_EXTENDED(PGMATRIX pMatResult, PGMATRIX pMatA, BOOL FlagTransposeA, PGMATRIX pMatB, BOOL FlagTransposeB, PGMATRIX pMatC, BOOL FlagTransposeC, PGMATRIX pMatDummy) ;

#define GMATRIX_TRIPLEMULTIPLY_ADD(MatResult, MatA, MatB, MatC, MatDummy) PGMATRIX_TRIPLEMULTIPLY_ADD(&MatResult, &MatA, &MatB, &MatC, &MatDummy)
void PGMATRIX_TRIPLEMULTIPLY_ADD(PGMATRIX pMatResult, PGMATRIX pMatA, PGMATRIX pMatB, PGMATRIX pMatC, PGMATRIX pMatDummy) ;

#define GMATRIX_TRIPLEMULTIPLY_ADD_EXTENDED(MatResult, MatA, FlagTransposeA, MatB, FlagTransposeB, MatC, FlagTransposeC, MatDummy) PGMATRIX_TRIPLEMULTIPLY_ADD_EXTENDED(&MatResult, &MatA, FlagTransposeA, &MatB, FlagTransposeB, &MatC, FlagTransposeC, &MatDummy)

void PGMATRIX_TRIPLEMULTIPLY_ADD_EXTENDED(PGMATRIX pMatResult, PGMATRIX pMatA, int FlagTransposeA, PGMATRIX pMatB, int FlagTransposeB, PGMATRIX pMatC, int FlagTransposeC, PGMATRIX pMatDummy);

#define GMATRIX_SUBMATRIX_COPY(Mat,nl,nc,MatOrigin) PGMATRIX_SUBMATRIX_COPY(&Mat,nl,nc,&MatOrigin);
void PGMATRIX_SUBMATRIX_COPY(PGMATRIX pMat, int nl, int nc, PGMATRIX pMatOrigin);

#define GMATRIX_SUBMATRIX_ADD(Mat,nl,nc,MatOrigin) PGMATRIX_SUBMATRIX_ADD(&Mat,nl,nc,&MatOrigin);
void PGMATRIX_SUBMATRIX_ADD(PGMATRIX pMat, int nl, int nc, PGMATRIX pMatOrigin);

#define GMATRIX_SUBMATRIX_COPY_EXTENDED(Mat,nl,nc,MatOrigin,FlagTransposeOrigin) PGMATRIX_SUBMATRIX_COPY_EXTENDED(&Mat,nl,nc,&MatOrigin,FlagTransposeOrigin);
void PGMATRIX_SUBMATRIX_COPY_EXTENDED(PGMATRIX pMat, int nl, int nc, PGMATRIX pMatOrigin, int FlagTransposeOrigin);

#define GMATRIX_SUBMATRIX_FILL(Mat,nlb,nle,ncb,nce,Value) PGMATRIX_SUBMATRIX_FILL(&Mat,nlb,nle,ncb,nce,Value)
void PGMATRIX_SUBMATRIX_FILL(PGMATRIX pMat,int nlb,int nle,int ncb,int nce,GMATRIX_FLOAT Value);

#define GMATRIX_SUBMATRIX_MULTIPLY_CONST(Mat,nlb,nle,ncb,nce,Value) PGMATRIX_SUBMATRIX_MULTIPLY_CONST(&Mat,nlb,nle,ncb,nce,Value)
void PGMATRIX_SUBMATRIX_MULTIPLY_CONST(PGMATRIX pMat,int nlb,int nle,int ncb,int nce,GMATRIX_FLOAT Value);

#ifdef __cplusplus
}
#endif

#endif // GMATRIX_H
