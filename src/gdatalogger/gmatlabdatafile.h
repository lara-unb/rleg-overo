/*****************************************************************************
// File: gMATLABDataFile.c 
// Contents: Heading for gMATLABDataFile.c. 
// Author: G. A. Borges.
*****************************************************************************/

#ifndef GMATLABDATAFILE_H
#define GMATLABDATAFILE_H

#define TRUE 1
#define FALSE 0

typedef struct{
	FILE *fp;
	char FileName[200];
	int FlagStillNotSaved;
} GMATLABDATAFILECONFIG, *PGMATLABDATAFILECONFIG;

int gMATLABDataFile_OpenWrite(PGMATLABDATAFILECONFIG pGMatlabDataFileConfig, char *filename, char *dirname);
int gMATLABDataFile_OpenRead(PGMATLABDATAFILECONFIG pGMatlabDataFileConfig, char *filename, char *dirname);
void gMATLABDataFile_Close(PGMATLABDATAFILECONFIG pGMatlabDataFileConfig);
int gMATLABDataFile_SaveVector(PGMATLABDATAFILECONFIG pGMatlabDataFileConfig, const char *varname, double *v, long nlin);
int gMATLABDataFile_SaveMatrix(PGMATLABDATAFILECONFIG pGMatlabDataFileConfig, const char *varname, double **m, long nlin, long ncol);

#endif
