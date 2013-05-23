/*****************************************************************************
// File: gMATLABDataFile.c 
// Contents: Functions for writing in .mat files (matlab) version 4.0.
// Author: G. A. Borges.
*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "gmatlabdatafile.h"

typedef struct{
	long type;
	long mrows;
	long ncols;
	long imagf;
	long namlen;
} MATLAB_DATAHEAD, *PMATLAB_DATAHEAD;

int gMATLABDataFile_OpenWrite(PGMATLABDATAFILECONFIG pGMatlabDataFileConfig, char *filename, char *dirname)
{
	pGMatlabDataFileConfig->FileName[0] = '\0';
	if(dirname!=NULL)
		sprintf(pGMatlabDataFileConfig->FileName,"%s",dirname);
	strcat(pGMatlabDataFileConfig->FileName,filename);
	pGMatlabDataFileConfig->FlagStillNotSaved = TRUE;

	pGMatlabDataFileConfig->fp = fopen(pGMatlabDataFileConfig->FileName, "w");
	if(pGMatlabDataFileConfig->fp==NULL){
		printf("\n pGMatlabDataFileConfig->fp==NULL: %s",pGMatlabDataFileConfig->FileName);
		return FALSE;
	}
	rewind(pGMatlabDataFileConfig->fp);

	return TRUE;

}

int gMATLABDataFile_OpenRead(PGMATLABDATAFILECONFIG pGMatlabDataFileConfig, char *filename, char *dirname)
{
	pGMatlabDataFileConfig->FileName[0] = '\0';
	if(dirname!=NULL)
		sprintf(pGMatlabDataFileConfig->FileName,"%s",dirname);
	strcat(pGMatlabDataFileConfig->FileName,filename);
	pGMatlabDataFileConfig->FlagStillNotSaved = TRUE;

	pGMatlabDataFileConfig->fp = fopen(pGMatlabDataFileConfig->FileName, "r");
	if(pGMatlabDataFileConfig->fp==NULL){
		return FALSE;
	}
	rewind(pGMatlabDataFileConfig->fp);

	return TRUE;

}

void gMATLABDataFile_Close(PGMATLABDATAFILECONFIG pGMatlabDataFileConfig)
{
	fclose(pGMatlabDataFileConfig->fp);

}

int gMATLABDataFile_SaveVector(PGMATLABDATAFILECONFIG pGMatlabDataFileConfig, const char *varname, double *v, long nlin)
{
	MATLAB_DATAHEAD DataHead;

	DataHead.type  = (long)(0);		
	DataHead.mrows = (long)(nlin);	
	DataHead.ncols = (long)(1);
	DataHead.imagf = (long)(0);
	DataHead.namlen = (long)(strlen(varname)+1);

	if(! fwrite(&DataHead, sizeof(MATLAB_DATAHEAD), 1, pGMatlabDataFileConfig->fp) ){
		return FALSE;
	}

	if(! fwrite(varname, sizeof(char), (strlen(varname)+1), pGMatlabDataFileConfig->fp)){
		return FALSE;
	}

	if(! fwrite(v, sizeof(double), nlin, pGMatlabDataFileConfig->fp)){
		return FALSE;
	}

	pGMatlabDataFileConfig->FlagStillNotSaved = FALSE;

	return(TRUE);

}

int gMATLABDataFile_SaveMatrix(PGMATLABDATAFILECONFIG pGMatlabDataFileConfig, const char *varname, double **m, long nlin, long ncol)
{
	MATLAB_DATAHEAD DataHead;
	int nl,nc;
	double *vcol;

	DataHead.type  = (long)(0);		
	DataHead.mrows = (long)(nlin);	
	DataHead.ncols = (long)(ncol);
	DataHead.imagf = (long)(0);
	DataHead.namlen = (long)(strlen(varname)+1);

	if(! fwrite(&DataHead, sizeof(MATLAB_DATAHEAD), 1, pGMatlabDataFileConfig->fp) ){
		return FALSE;
	}

	if(! fwrite(varname, sizeof(char), (strlen(varname)+1), pGMatlabDataFileConfig->fp)){
		return FALSE;
	}

	vcol = (double*)malloc(nlin*sizeof(double));

	for(nc=0;nc<ncol;++nc){
		for(nl=0;nl<nlin;++nl){
			vcol[nl] = m[nl][nc];
		}
		if(! fwrite(vcol, sizeof(double), nlin, pGMatlabDataFileConfig->fp)){
			free(vcol);
			return FALSE;
		}
	}
	free(vcol);

	pGMatlabDataFileConfig->FlagStillNotSaved = FALSE;

	return(TRUE);

}
