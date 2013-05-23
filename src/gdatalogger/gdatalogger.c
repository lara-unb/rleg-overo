/*****************************************************************************
// File: GDataLogger.c 
// Contents: Functions for recording data points in matlab format.
// Author: G. A. Borges.
*****************************************************************************/
#if DATALOGGER_COMPILE_FOR_CMEX
#include "mex.h"
#endif

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h> 
#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <errno.h>

#if DATALOGGER_COMPILE_FOR_XENOMAI
	#include <native/heap.h>
	#include <native/types.h> 
#endif

#include "gqueue.h"
#include "gmatlabdatafile.h"
#include "gdatalogger.h"

static GDATALOGGERIPC_SHM *pgDataLoggerIPC_SHM = NULL;
static int gDataLoggerIPC_SHM_flagmodeserver = 1;

#if DATALOGGER_COMPILE_FOR_XENOMAI
RT_HEAP gDataLoggerIPC_SHM_hd;
#else
static int gDataLoggerIPC_SHM_fd = 0;
#endif

/**********************************************************************************************
***** gDataLogger: Init
**********************************************************************************************/
int gDataLogger_Init(PGDATALOGGER pgDataLogger, char *filename, char *dirname)
{
	int i;
	int first = 0;
#if DATALOGGER_COMPILE_FOR_XENOMAI
	int err;
#else
	int status = 0;
#endif

	/* GQueue */
	if(pgDataLogger!=NULL){
		for (i=0;i<GDATALOGGER_MAXVARIABLES;++i){
			sprintf(pgDataLogger->Variables[i].VariableName," ");
			sprintf(pgDataLogger->Variables[i].VariableUnit," ");
			pgDataLogger->Variables[i].CircularQueue = NULL;
			pgDataLogger->Variables[i].GMatlabDataFileIndex = 0;
			pgDataLogger->Variables[i].Nc = 1;
			pgDataLogger->Variables[i].Nr = 1;
            pgDataLogger->Variables[i].HasBeenWritten = 0;
		}
		pgDataLogger->m_NumberOfVariables = 0;
	}

	/* IPC */
#if DATALOGGER_COMPILE_FOR_XENOMAI
	if(pgDataLogger!=NULL){
		/* cria memoria compartilhada */
		int heapsize;
		if(sizeof(GDATALOGGERIPC_SHM) > ((256*1024))){
			heapsize = sizeof(GDATALOGGERIPC_SHM);
		}
		else{
			heapsize = ((256*1024));
		}
		
		if((err=rt_heap_create(&gDataLoggerIPC_SHM_hd,GDATALOGGER_IPC_STATEFILE,sizeof(GDATALOGGERIPC_SHM),H_SHARED))!=0){
			printf("gDataLogger_Init: Could not create xenomai heap object (return %i)\n", err);
			printf("\n EEXIST = %i",EEXIST);
			printf("\n EINVAL = %i",EINVAL);
			printf("\n ENOMEM = %i",ENOMEM);
			printf("\n EPERM = %i",EPERM);
			printf("\n EIDRM = %i",EIDRM);
			printf("\n ENOSYS = %i",ENOSYS);
			if(err==-EEXIST){
				err=rt_heap_delete(&gDataLoggerIPC_SHM_hd);
				printf("gDataLogger_Init: Deleting xenomai heap object (return %i)\n", err);
			}
			return FALSE;
		}

		if(rt_heap_alloc(&gDataLoggerIPC_SHM_hd,sizeof(GDATALOGGERIPC_SHM),TM_NONBLOCK,(void**)&pgDataLoggerIPC_SHM)!=0){
			printf("gDataLogger_Init: Could not allocated xenomai heap object. (return %i)\n", err);
			return FALSE;
		}

		first = 1; /* We are the first instance */
		gDataLoggerIPC_SHM_flagmodeserver = 1;
	}
	else{
		/* abre memoria compartilhada */
		if((err=rt_heap_bind(&gDataLoggerIPC_SHM_hd,GDATALOGGER_IPC_STATEFILE,TM_NONBLOCK))!=0){
			/* printf("gDataLogger_Init: Could not create xenomai heap object (return %i)\n", err); */
/*			printf("\n EEXIST = %i",EEXIST);
			printf("\n EINVAL = %i",EINVAL);
			printf("\n ENOMEM = %i",ENOMEM);
			printf("\n EPERM = %i",EPERM);
			printf("\n EIDRM = %i",EIDRM);
			printf("\n ENOSYS = %i",ENOSYS);*/
			return FALSE;
		}

		if(rt_heap_alloc(&gDataLoggerIPC_SHM_hd,0,TM_NONBLOCK,(void**)&pgDataLoggerIPC_SHM)!=0){
			printf("gDataLogger_Init: Could not allocated xenomai heap object. (return %i)\n", err);
			return FALSE;
		}

		gDataLoggerIPC_SHM_flagmodeserver = 0;
	}

#else
	/* Try to open the shm instance with  O_EXCL, this tests if the shm is already opened by someone else */
	if((gDataLoggerIPC_SHM_fd = shm_open(GDATALOGGER_IPC_STATEFILE, (O_CREAT | O_RDWR),(S_IREAD | S_IWRITE))) < 0) {
		/* Try to open the shm instance normally and share it with existing clients */
		printf("gDataLogger_Init: Could not create shm object. %s\n", strerror(errno));
		return FALSE;
	}	
	if(pgDataLogger!=NULL){
		gDataLoggerIPC_SHM_flagmodeserver = 1;
		first = 1;
	} else{
		gDataLoggerIPC_SHM_flagmodeserver = 0;
		first = 0;
	}
/*	if((gDataLoggerIPC_SHM_fd = shm_open(GDATALOGGER_IPC_STATEFILE, (O_CREAT | O_EXCL | O_RDWR),(S_IREAD | S_IWRITE))) > 0 ) {
*/		  first = 1; /* We are the first instance */
/*		  gDataLoggerIPC_SHM_flagmodeserver = 1;
	}
	else if((gDataLoggerIPC_SHM_fd = shm_open(GDATALOGGER_IPC_STATEFILE, (O_CREAT | O_RDWR),(S_IREAD | S_IWRITE))) < 0) {
*/		/* Try to open the shm instance normally and share it with existing clients */
/*		printf("gDataLogger_Init: Could not create shm object. %s\n", strerror(errno));
		return FALSE;
	} 
*/	/* Set the size of the SHM to be the size of the struct. */
	status = ftruncate(gDataLoggerIPC_SHM_fd, sizeof(GDATALOGGERIPC_SHM));

	/* Connect the conf pointer to set to the shared memory area, with desired permissions */
	if((pgDataLoggerIPC_SHM =  mmap(0, sizeof(GDATALOGGERIPC_SHM), (PROT_READ | PROT_WRITE), MAP_SHARED, gDataLoggerIPC_SHM_fd, 0)) == MAP_FAILED) {
		return FALSE;
	}

	/* Set permission so all can read/write to shared memory */
    status = fchmod(gDataLoggerIPC_SHM_fd, (S_IRUSR|S_IWUSR|S_IXUSR|S_IRGRP|S_IWGRP|S_IXGRP|S_IROTH|S_IWOTH|S_IXOTH));

#endif
	
	if(first){
		pgDataLoggerIPC_SHM->Flag = GDATALOGGER_IPC_FLAGIDDLE;
	}

	/* GMatlabDataFile */
	if((pgDataLogger!=NULL)&&(filename!=NULL)){
		return(gMATLABDataFile_OpenWrite(&pgDataLogger->GMatlabDataFileConfig, filename, dirname));
	}
	
	return TRUE;
}

/**********************************************************************************************
***** gDataLogger: DeclareVariable
**********************************************************************************************/
int gDataLogger_DeclareVariable(PGDATALOGGER pgDataLogger, char *varname, char *varunit, int nrows, int ncols, int queuesize)
{
	if(pgDataLogger->m_NumberOfVariables>=GDATALOGGER_MAXVARIABLES-1){
		return(FALSE);
	}
	queuesize = queuesize * nrows * ncols; /* no caso, o tamanho da fila eh calculado em numero de elementos double. */
	sprintf(pgDataLogger->Variables[pgDataLogger->m_NumberOfVariables].VariableName,"%s",varname);
	sprintf(pgDataLogger->Variables[pgDataLogger->m_NumberOfVariables].VariableUnit,"%s",varunit);
	pgDataLogger->Variables[pgDataLogger->m_NumberOfVariables].CircularQueue = (double *) malloc(queuesize * nrows * ncols * sizeof(double));
	if(pgDataLogger->Variables[pgDataLogger->m_NumberOfVariables].CircularQueue==NULL){
		return(FALSE);
	}
	pgDataLogger->Variables[pgDataLogger->m_NumberOfVariables].Nr = nrows;
	pgDataLogger->Variables[pgDataLogger->m_NumberOfVariables].Nc = ncols;
	gQUEUE_Init(&pgDataLogger->Variables[pgDataLogger->m_NumberOfVariables].CircularQueueControl, queuesize, QUEUE_MAXDESTINATIONS);
	pgDataLogger->Variables[pgDataLogger->m_NumberOfVariables].GMatlabDataFileIndex = 0;
	
	++pgDataLogger->m_NumberOfVariables;

	return(TRUE);
}


/**********************************************************************************************
***** gDataLogger: InsertVariable
**********************************************************************************************/
int gDataLogger_InsertVariable(PGDATALOGGER pgDataLogger, char *varname, double *varvalue)
{
	int queueindex,varindex,n;
	long int nr,nc;
	int FlagQueueFull;
	
	/* Procura pela variavel */
	varindex = -1;
	for(n=0;n<pgDataLogger->m_NumberOfVariables;++n){
		if(strcmp(varname,pgDataLogger->Variables[n].VariableName)==0){
			varindex = n;
			break;
		}
	}
	if(varindex<0){
		return(FALSE);
	}

    if(pgDataLogger->Variables[varindex].HasBeenWritten == 1) pgDataLogger->Variables[varindex].HasBeenWritten = 2; // Second write
    if(pgDataLogger->Variables[varindex].HasBeenWritten == 0) pgDataLogger->Variables[varindex].HasBeenWritten = 1; // First write (1 data point)

	/* Insere o conteudo */
	for(nr=0;nr<pgDataLogger->Variables[varindex].Nr;++nr){
		for(nc=0;nc<pgDataLogger->Variables[varindex].Nc;++nc){
			FlagQueueFull = gQUEUE_RequestWriteIndex(&pgDataLogger->Variables[varindex].CircularQueueControl, &queueindex);

			pgDataLogger->Variables[varindex].CircularQueue[queueindex] = varvalue[nr+(nc)*pgDataLogger->Variables[varindex].Nr];

			if(!FlagQueueFull){
				/* The queue is full. Test if it is the reading head associated to matlab data files. */
				if(pgDataLogger->Variables[varindex].CircularQueueControl.FlagFull[QUEUE_DESTINATION_FILE]==TRUE){
					/* Empty in matlab data file. */
					gDataLogger_MatfileUpdate(pgDataLogger);
					/* Clear flag: */
					pgDataLogger->Variables[varindex].CircularQueueControl.FlagFull[QUEUE_DESTINATION_FILE] = FALSE;
				}
			}
		}
	}

	return(TRUE);
}

/**********************************************************************************************
***** gDataLogger: Update
**********************************************************************************************/
int gDataLogger_MatfileUpdate(PGDATALOGGER pgDataLogger)
{
	int QueueIndex,nvar,datasize,i;
	double *v;
	char matvarname[70];

	/********** MATFILE **********/
	/* Esvaziar queues de todas as variaveis que vao para o arquivo: */
	for(nvar=0;nvar<pgDataLogger->m_NumberOfVariables;++nvar){
		/* Metodo otimizado, que salva em uma variavel MATLAB todas as ultimas leituras. */
		if(!gQUEUE_GetNumberOfUnreadData(&pgDataLogger->Variables[nvar].CircularQueueControl, QUEUE_DESTINATION_FILE, &datasize)){
			return(FALSE);
		}
		if(datasize>0){ /* Existem dados a serem salvos. */
			v = (double *) malloc(datasize * sizeof(double));
			for(i=0;i<datasize;++i){
				gQUEUE_RequestReadIndex(&pgDataLogger->Variables[nvar].CircularQueueControl, QUEUE_DESTINATION_FILE, &QueueIndex);
				v[i] = pgDataLogger->Variables[nvar].CircularQueue[QueueIndex];
			}
			sprintf(matvarname,"%s_%li_%li",pgDataLogger->Variables[nvar].VariableName,pgDataLogger->Variables[nvar].GMatlabDataFileIndex,pgDataLogger->Variables[nvar].GMatlabDataFileIndex+datasize-1);
			gMATLABDataFile_SaveVector(&pgDataLogger->GMatlabDataFileConfig, matvarname, v, datasize);
			free(v);
			pgDataLogger->Variables[nvar].GMatlabDataFileIndex += datasize;
		}
	}

	return(TRUE);
}

int gDataLogger_IPCUpdate(PGDATALOGGER pgDataLogger)
{
	int QueueIndex,nvar,n,i;

	/********** IPC **********/
	if(gDataLoggerIPC_SHM_flagmodeserver==0){
		return FALSE;
	}
	
	if(pgDataLoggerIPC_SHM->Flag == GDATALOGGER_IPC_FLAGREQUESTDATA){
		
		/* Procura pela variavel */
		nvar = -1;
		for(n=0;n<pgDataLogger->m_NumberOfVariables;++n){
			if(strcmp(pgDataLoggerIPC_SHM->VariableName,pgDataLogger->Variables[n].VariableName)==0){
				nvar = n;
				break;
			}
		}
		if(nvar<0){
			pgDataLoggerIPC_SHM->Flag = GDATALOGGER_IPC_FLAGNOTEXIST;
			return FALSE; 
		}

		/* Metodo otimizado, que salva em uma variavel MATLAB todas as ultimas leituras. */
		if(!gQUEUE_GetNumberOfUnreadData(&pgDataLogger->Variables[nvar].CircularQueueControl, QUEUE_DESTINATION_IPC, &pgDataLoggerIPC_SHM->QueueSize)){
			pgDataLoggerIPC_SHM->QueueSize = 0;
		}
		if(pgDataLoggerIPC_SHM->QueueSize>0){ /* Existem dados a serem salvos. */
			for(i=0;i<pgDataLoggerIPC_SHM->QueueSize;++i){
				gQUEUE_RequestReadIndex(&pgDataLogger->Variables[nvar].CircularQueueControl, QUEUE_DESTINATION_IPC, &QueueIndex);
				pgDataLoggerIPC_SHM->QueueData[i] = pgDataLogger->Variables[nvar].CircularQueue[QueueIndex];
			}
		}
		
		sprintf(pgDataLoggerIPC_SHM->VariableUnit, "%s", pgDataLogger->Variables[nvar].VariableUnit);
		pgDataLoggerIPC_SHM->Flag = GDATALOGGER_IPC_FLAGDATAAVAILABLE;
	}

	return(TRUE);
}

/**********************************************************************************************
***** gDataLogger: gDataLogger_IPC_RetrieveVariable
**********************************************************************************************/
int gDataLogger_IPC_RetrieveVariable(char *varname, char *varunit, double *pbuffer, int *bufferlen)
{
	int counter;
	
	sprintf(pgDataLoggerIPC_SHM->VariableName, "%s", varname);
	pgDataLoggerIPC_SHM->Flag = GDATALOGGER_IPC_FLAGREQUESTDATA;
	
/*	printf("\ngDataLogger_IPC_RetrieveVariable: solicitação feita pela variavel %s",varname); */

	counter = 0;
	while(pgDataLoggerIPC_SHM->Flag == GDATALOGGER_IPC_FLAGREQUESTDATA){
		usleep(10000);
		if(++counter > 100){
			*bufferlen = 0;
			pgDataLoggerIPC_SHM->Flag = GDATALOGGER_IPC_FLAGIDDLE;
			/* printf("\ngDataLogger_IPC_RetrieveVariable: solicitação não atendida"); */
			return GDATALOGGER_IPC_FLAGTIMEOUT;
		}
	}
	
	if(pgDataLoggerIPC_SHM->Flag==GDATALOGGER_IPC_FLAGDATAAVAILABLE){
		memcpy(pbuffer,pgDataLoggerIPC_SHM->QueueData, pgDataLoggerIPC_SHM->QueueSize * sizeof(double));
		sprintf(varunit, "%s", pgDataLoggerIPC_SHM->VariableUnit);
		*bufferlen = pgDataLoggerIPC_SHM->QueueSize;
		pgDataLoggerIPC_SHM->Flag = GDATALOGGER_IPC_FLAGIDDLE;
		return GDATALOGGER_IPC_FLAGDATAAVAILABLE;
	}

	if(pgDataLoggerIPC_SHM->Flag==GDATALOGGER_IPC_FLAGNOTEXIST){
		*bufferlen = 0;
		pgDataLoggerIPC_SHM->Flag = GDATALOGGER_IPC_FLAGIDDLE;
		return GDATALOGGER_IPC_FLAGNOTEXIST;
	}
	
	*bufferlen = 0;
	pgDataLoggerIPC_SHM->Flag = GDATALOGGER_IPC_FLAGIDDLE;
	return GDATALOGGER_IPC_FLAGIDDLE;
}


/**********************************************************************************************
***** gDataLogger: Close
**********************************************************************************************/
int gDataLogger_Close(PGDATALOGGER pgDataLogger)
{
	int nvar,datasize;
	double v;
    double *value;
	char matvarname[70];

	/* Garante salvar ultimos dados inseridos: */
	if(pgDataLogger!=NULL){
		gDataLogger_MatfileUpdate(pgDataLogger);

		/* Se for um unico escalar, trata de forma especial: */
        for(nvar=0;nvar<pgDataLogger->m_NumberOfVariables;++nvar){
            if((pgDataLogger->Variables[nvar].Nc == 1)&&(pgDataLogger->Variables[nvar].Nr == 1)){
                    if(pgDataLogger->Variables[nvar].HasBeenWritten == 1){
                        value = (double *) malloc(sizeof(double));
                        value[0] = pgDataLogger->Variables[nvar].CircularQueue[0];
                        sprintf(matvarname,"%s_%li_%li",pgDataLogger->Variables[nvar].VariableName, (long int)0, (long int)0);
                        gMATLABDataFile_SaveVector(&pgDataLogger->GMatlabDataFileConfig, matvarname, value, 1);
                        free(value);
                        pgDataLogger->Variables[nvar].GMatlabDataFileIndex = 1;
                    }
            }
        }

		/* Cria variaveis que indicam o numero de cada variavel salva: */
		for(nvar=0;nvar<pgDataLogger->m_NumberOfVariables;++nvar){
			v = (double)(pgDataLogger->Variables[nvar].GMatlabDataFileIndex);
			datasize = 1;
			sprintf(matvarname,"%s_size",pgDataLogger->Variables[nvar].VariableName);
			gMATLABDataFile_SaveVector(&pgDataLogger->GMatlabDataFileConfig, matvarname, &v, datasize);
		}

		/* Salva o numero de linhas: */
		for(nvar=0;nvar<pgDataLogger->m_NumberOfVariables;++nvar){
			v = (double)(pgDataLogger->Variables[nvar].Nr);
			datasize = 1;
			sprintf(matvarname,"%s_nr",pgDataLogger->Variables[nvar].VariableName);
			gMATLABDataFile_SaveVector(&pgDataLogger->GMatlabDataFileConfig, matvarname, &v, datasize);
		}

		/* Salva o numero de colunas: */
		for(nvar=0;nvar<pgDataLogger->m_NumberOfVariables;++nvar){
			v = (double)(pgDataLogger->Variables[nvar].Nc);
			datasize = 1;
			sprintf(matvarname,"%s_nc",pgDataLogger->Variables[nvar].VariableName);
			gMATLABDataFile_SaveVector(&pgDataLogger->GMatlabDataFileConfig, matvarname, &v, datasize);
		}

		/* Fecha o arquivo. */
		gMATLABDataFile_Close(&pgDataLogger->GMatlabDataFileConfig);

		/* Deletar as filas: */
		for(nvar=0;nvar<pgDataLogger->m_NumberOfVariables;++nvar){
			sprintf(pgDataLogger->Variables[nvar].VariableName," ");
			sprintf(pgDataLogger->Variables[nvar].VariableUnit," ");
			free(pgDataLogger->Variables[nvar].CircularQueue);
			pgDataLogger->Variables[nvar].CircularQueue = NULL;
			pgDataLogger->Variables[nvar].GMatlabDataFileIndex = 0;
		}
		pgDataLogger->m_NumberOfVariables = 0;
	}
	
	/* IPC: */
#if DATALOGGER_COMPILE_FOR_XENOMAI
	if(gDataLoggerIPC_SHM_flagmodeserver){
		rt_heap_free(&gDataLoggerIPC_SHM_hd,pgDataLoggerIPC_SHM);
		rt_heap_delete(&gDataLoggerIPC_SHM_hd);
	} else{
		rt_heap_unbind(&gDataLoggerIPC_SHM_hd);
	}
#else
	munmap(pgDataLoggerIPC_SHM, sizeof(GDATALOGGERIPC_SHM));
	close(gDataLoggerIPC_SHM_fd);
	shm_unlink(GDATALOGGER_IPC_STATEFILE);
#endif
	
	return(TRUE);
}

/****************************************************************
**** 
**** Scope
**** 
****************************************************************/
/*
int GDataLogger::ScopeCreate(CWnd *pIDCWindow, int nIDC, PGDATALOGGERSCOPEPARAMETERS pDataLoggerScopeParameters)
{
	// Verify whether there is available Scope:
	if (pgDataLogger->m_NumberOfScopes>=(GDATALOGGER_MAXSCOPES-1)){
		return(FALSE);
	}

	// nIDC, pWnd and pDC:
	pgDataLogger->Scopes[m_NumberOfScopes].nIDC = nIDC;
	pgDataLogger->Scopes[m_NumberOfScopes].pWnd = pIDCWindow;
	pgDataLogger->Scopes[m_NumberOfScopes].pDC  = pgDataLogger->Scopes[m_NumberOfScopes].pWnd->GetDC();
	pgDataLogger->Scopes[m_NumberOfScopes].NumberOfInsertedVariables = 0;

	// Parameters:
	memcpy(&pgDataLogger->Scopes[m_NumberOfScopes].Parameters,pDataLoggerScopeParameters,sizeof(GDATALOGGERSCOPEPARAMETERS));

	// Previously ploted? No
	for(int i=0;i<GDATALOGGER_MAXVARIABLES;++i){
		pgDataLogger->Scopes[m_NumberOfScopes].FirstPointPloted[i] = FALSE;
	}

	// Compute rectangles:
	CRect Rect;
	TEXTMETRIC TextMetric;

	pgDataLogger->Scopes[m_NumberOfScopes].pWnd->GetWindowRect(&Rect);
	pgDataLogger->Scopes[m_NumberOfScopes].Parameters.RectDCPix.left   = 0;
	pgDataLogger->Scopes[m_NumberOfScopes].Parameters.RectDCPix.top    = 0;
	pgDataLogger->Scopes[m_NumberOfScopes].Parameters.RectDCPix.right  = Rect.right - Rect.left;
	pgDataLogger->Scopes[m_NumberOfScopes].Parameters.RectDCPix.bottom = Rect.bottom - Rect.top;

	pgDataLogger->Scopes[m_NumberOfScopes].pDC->GetTextMetrics(&TextMetric); 
	pgDataLogger->Scopes[m_NumberOfScopes].Parameters.RectPlotPix.left   = 
		+ pgDataLogger->Scopes[m_NumberOfScopes].Parameters.RectDCPix.left 
		+ 2*TextMetric.tmHeight // Numeros dos eixos
		+ 1*TextMetric.tmHeight + 2; // Label
	pgDataLogger->Scopes[m_NumberOfScopes].Parameters.RectPlotPix.top    = pgDataLogger->Scopes[m_NumberOfScopes].Parameters.RectDCPix.top + TextMetric.tmExternalLeading + TextMetric.tmHeight + 2;
	pgDataLogger->Scopes[m_NumberOfScopes].Parameters.RectPlotPix.right  = pgDataLogger->Scopes[m_NumberOfScopes].Parameters.RectDCPix.right - (2*TextMetric.tmMaxCharWidth + 2);
	pgDataLogger->Scopes[m_NumberOfScopes].Parameters.RectPlotPix.bottom = pgDataLogger->Scopes[m_NumberOfScopes].Parameters.RectDCPix.bottom - TextMetric.tmExternalLeading - 2*TextMetric.tmHeight - 2;

	// Fonts, Pens, Brushs.
	pgDataLogger->Scopes[m_NumberOfScopes].Parameters.TextFont.CreateFont( 
			-10, 
			0, 
			0, 
			0, 
			FW_SEMIBOLD, 
			FALSE, 
			FALSE, 
			0, 
			ANSI_CHARSET, 
			OUT_DEFAULT_PRECIS, 
			CLIP_DEFAULT_PRECIS, 
			DEFAULT_QUALITY, 
			DEFAULT_PITCH | FF_SWISS, 
			"Arial");
	
	pgDataLogger->Scopes[m_NumberOfScopes].Parameters.TextFontRotated.CreateFont( 
			-10, 
			0, 
			900, 
			0, 
			FW_SEMIBOLD, 
			FALSE, 
			FALSE, 
			0, 
			ANSI_CHARSET, 
			OUT_DEFAULT_PRECIS, 
			CLIP_DEFAULT_PRECIS, 
			DEFAULT_QUALITY, 
			DEFAULT_PITCH | FF_SWISS, 
			"Arial");

	pgDataLogger->Scopes[m_NumberOfScopes].Parameters.ScopePen.CreatePen( 
		PS_SOLID, 
		1, 
		pgDataLogger->Scopes[m_NumberOfScopes].Parameters.ScopeColor);
	
	pgDataLogger->Scopes[m_NumberOfScopes].Parameters.ScopeBrush.CreateSolidBrush(
		pgDataLogger->Scopes[m_NumberOfScopes].Parameters.ScopeColor);

	pgDataLogger->Scopes[m_NumberOfScopes].Parameters.BackgroundBrush.CreateSolidBrush(
		pgDataLogger->Scopes[m_NumberOfScopes].Parameters.BackgroundColor);

	// Increment the number of scopes:
	++pgDataLogger->m_NumberOfScopes;

	return(TRUE);
}

void GDataLogger::ScopeClose(void)
{
	int i;

	for(int nscope=0;nscope <pgDataLogger->m_NumberOfScopes;++nscope){
		// Release DC:
		pgDataLogger->Scopes[nscope].pWnd->ReleaseDC(pgDataLogger->Scopes[nscope].pDC);
		// Clear inserted variables
		pgDataLogger->Scopes[nscope].NumberOfInsertedVariables = 0;
		// Default time source index
		pgDataLogger->Scopes[nscope].TimeSourceIndex = -1;
		// 
		for(i=0;i<GDATALOGGER_MAXVARIABLES;++i){
			pgDataLogger->Scopes[nscope].FirstPointPloted[i] = FALSE;
		}
	}
}

void GDataLogger::ScopeSetDefaultParameters(PGDATALOGGERSCOPEPARAMETERS pDataLoggerScopeParameters)
{
	pDataLoggerScopeParameters->X_DivNumber = 9;
	pDataLoggerScopeParameters->Y_DivNumber = 7;
	pDataLoggerScopeParameters->X_Offset   = 0.0;
	pDataLoggerScopeParameters->X_DivScale = 1.0;
	pDataLoggerScopeParameters->Y_Offset   = 0.0;
	pDataLoggerScopeParameters->Y_DivScale = 1.0;
	pDataLoggerScopeParameters->BackgroundColor = RGB(255,255,255);
	pDataLoggerScopeParameters->ScopeColor = RGB(0,0,255);
	pDataLoggerScopeParameters->TextColor = RGB(128,0,64);
}

int GDataLogger::ScopeDraw(int nIDC, int FlagErase)
{
	int nscope;
	
	// Get scope index from IDC
	if((nscope = pgDataLogger->ScopeGetIndex(nIDC))<0){
		return(FALSE);
	}
	if(FlagErase)
		pgDataLogger->ScopeDrawAxis(nscope);
	pgDataLogger->ScopeDrawPlotVariables(nscope);
	return(TRUE);
}

int GDataLogger::ScopeInsertVariable(int nIDC, char *varname)
{
	int scopeindex,varindex;
	
	// Get scope index from IDC
	if((scopeindex = pgDataLogger->ScopeGetIndex(nIDC))<0){
		return(FALSE);
	}
	
	if (pgDataLogger->Scopes[scopeindex].NumberOfInsertedVariables>=(GDATALOGGER_MAXVARIABLES-1)){
		return(FALSE);
	}

	// Get variable index from varname
	if((varindex = pgDataLogger->VariableGetIndex(varname))<0){
		return(FALSE);
	}

	pgDataLogger->Scopes[scopeindex].InsertedVariablesIndex[pgDataLogger->Scopes[scopeindex].NumberOfInsertedVariables] = varindex;
	pgDataLogger->Scopes[scopeindex].FirstPointPloted[pgDataLogger->Scopes[scopeindex].NumberOfInsertedVariables] = FALSE;

	++pgDataLogger->Scopes[scopeindex].NumberOfInsertedVariables;

	return(TRUE);
}

int GDataLogger::ScopeDefineTimeSource(int nIDC, char *varname)
{
	int scopeindex,varindex;
	
	// Get scope index from IDC
	if((scopeindex = pgDataLogger->ScopeGetIndex(nIDC))<0){
		return(FALSE);
	}
	
	// Get variable index from varname
	if (varname != NULL){
		if((varindex = pgDataLogger->VariableGetIndex(varname))<0){
			return(FALSE);
		}
		pgDataLogger->Scopes[scopeindex].TimeSourceIndex = varindex;
	}
	else{
		pgDataLogger->Scopes[scopeindex].TimeSourceIndex = -1;
	}

	return(TRUE);
}

void TextOutRotated(CDC* pDC, const CString str, int x, int y, double angle)
{
   // convert angle to radian
   double pi = 3.141592654;
   double radian = pi * 2 / 360 * angle;
 
   // get the center of a not-rotated text
   CSize TextSize = pDC->GetTextExtent(str);
   CPoint center;
   center.x = TextSize.cx / 2;
   center.y = TextSize.cy / 2;
 
   // now calculate the center of the rotated text
   CPoint rcenter;
   rcenter.x = long(cos(radian) * center.x - sin(radian) * center.y);
   rcenter.y = long(sin(radian) * center.x + cos(radian) * center.y);
 
   // finally draw the text and move it to the center of the rectangle
   pDC->SetTextAlign(TA_BASELINE);
   pDC->SetBkMode(TRANSPARENT);
   pDC->TextOut(x - rcenter.x, y + rcenter.y, str);
}

int GDataLogger::ScopeDrawAxis(int nscope)
{
	CPen *pScopePen;
	CPen *pOldPen;
	CBrush *pScopeBrush;
	CBrush *pBackgroundBrush;
	CBrush *pOldBrush;
	CFont *pTextFont;
	CFont *pTextFontRotated;
	CFont *pOldFont;
	COLORREF *pTextColor;
	CRect *pRectPlotPix;
	CRect *pRectDCPix;
	double X_DivScale; 
	double Y_DivScale; 
	double X_Offset; 
	double Y_Offset; 
	int X_DivNumber;
	int	Y_DivNumber;

	double Xmin, Xmax, Ymin, Ymax;

	CString Strg;
	POINT Points[10];
	CRgn XArrow, YArrow;
	int i,j;

	CWnd *pWnd;
	CDC  *pDC;

	pWnd = pgDataLogger->Scopes[nscope].pWnd;
	pDC  = pgDataLogger->Scopes[nscope].pDC;
	pScopePen = &pgDataLogger->Scopes[nscope].Parameters.ScopePen;
	pScopeBrush = &pgDataLogger->Scopes[nscope].Parameters.ScopeBrush;
	pBackgroundBrush = &pgDataLogger->Scopes[nscope].Parameters.BackgroundBrush;
	pTextFont = &pgDataLogger->Scopes[nscope].Parameters.TextFont;
	pTextFontRotated = &pgDataLogger->Scopes[nscope].Parameters.TextFontRotated;
	pTextColor = &pgDataLogger->Scopes[nscope].Parameters.TextColor;
	pRectPlotPix = &pgDataLogger->Scopes[nscope].Parameters.RectPlotPix;
	pRectDCPix = &pgDataLogger->Scopes[nscope].Parameters.RectDCPix;
	X_DivScale = pgDataLogger->Scopes[nscope].Parameters.X_DivScale;
	Y_DivScale = pgDataLogger->Scopes[nscope].Parameters.Y_DivScale;
	X_Offset = pgDataLogger->Scopes[nscope].Parameters.X_Offset;
	Y_Offset = pgDataLogger->Scopes[nscope].Parameters.Y_Offset;
	X_DivNumber = pgDataLogger->Scopes[nscope].Parameters.X_DivNumber; 
	Y_DivNumber = pgDataLogger->Scopes[nscope].Parameters.Y_DivNumber; 

	Xmin = X_Offset - (X_DivNumber/2)* X_DivScale;
	Xmax = X_Offset + (X_DivNumber/2)* X_DivScale;
	Ymin = Y_Offset - (Y_DivNumber/2)* Y_DivScale;
	Ymax = Y_Offset + (Y_DivNumber/2)* Y_DivScale;

	// Save current drawing objects
	pOldPen = (CPen*) pDC->GetCurrentPen(); 
	pOldFont = (CFont*) pDC->GetCurrentFont(); 
	pOldBrush = (CBrush*) pDC->GetCurrentBrush(); 

	// Draw Background:
	pDC->SelectObject(pBackgroundBrush);
	pDC->Rectangle(*pRectDCPix);

	// Draw Axis: 
	pDC->SetTextColor(*pTextColor);
	pDC->SelectObject(pTextFont);
	pDC->SelectObject(pScopePen);
	pDC->SelectObject(pScopeBrush);
	pDC->MoveTo(pRectPlotPix->left,pRectPlotPix->bottom);
	pDC->LineTo(pRectPlotPix->right,pRectPlotPix->bottom);
	pDC->MoveTo(pRectPlotPix->left,pRectPlotPix->bottom);
	pDC->LineTo(pRectPlotPix->left,pRectPlotPix->top);
	
	Points[0].x = pRectPlotPix->right + 10;	Points[0].y = pRectPlotPix->bottom - 5;
	Points[1].x = pRectPlotPix->right + 15;	Points[1].y = pRectPlotPix->bottom;
	Points[2].x = pRectPlotPix->right + 10;	Points[2].y = pRectPlotPix->bottom + 5;
	XArrow.CreatePolygonRgn( Points, 3, WINDING);
	Points[0].x = pRectPlotPix->left - 5;	Points[0].y = pRectPlotPix->top - 10;
	Points[1].x = pRectPlotPix->left;		Points[1].y = pRectPlotPix->top - 15;
	Points[2].x = pRectPlotPix->left + 5;	Points[2].y = pRectPlotPix->top - 10;
	YArrow.CreatePolygonRgn( Points, 3, WINDING);
	pDC->MoveTo(pRectPlotPix->right,pRectPlotPix->bottom);
	pDC->LineTo(pRectPlotPix->right + 10,pRectPlotPix->bottom);
	pDC->MoveTo(pRectPlotPix->left,pRectPlotPix->top);
	pDC->LineTo(pRectPlotPix->left,pRectPlotPix->top - 11);
	pDC->SelectStockObject(BLACK_BRUSH);
	pDC->PaintRgn(&XArrow);
	pDC->PaintRgn(&YArrow);

	// Grades:
	pDC->SetTextAlign(TA_CENTER | TA_TOP);
	for (i=0; i<X_DivNumber; ++i){
		pDC->MoveTo(
			pRectPlotPix->left + (i*(pRectPlotPix->right-pRectPlotPix->left))/(X_DivNumber-1),
			pRectPlotPix->bottom-3);
		pDC->LineTo(
			pRectPlotPix->left + (i*(pRectPlotPix->right-pRectPlotPix->left))/(X_DivNumber-1),
			pRectPlotPix->bottom + 3);
		Strg.Format("%1.1f",Xmin + i*(Xmax-Xmin)/(X_DivNumber-1));
		pDC->TextOut(
			pRectPlotPix->left + (i*(pRectPlotPix->right-pRectPlotPix->left))/(X_DivNumber-1),
			pRectPlotPix->bottom + 3,
			Strg);
	}
	pDC->SetTextAlign(TA_RIGHT | TA_BASELINE);
	for (i=0; i<Y_DivNumber; ++i){
		pDC->MoveTo(pRectPlotPix->left - 3,
					pRectPlotPix->bottom - (i*(pRectPlotPix->bottom-pRectPlotPix->top))/(Y_DivNumber-1));
		pDC->LineTo(pRectPlotPix->left + 3,
					pRectPlotPix->bottom - (i*(pRectPlotPix->bottom-pRectPlotPix->top))/(Y_DivNumber-1));
		Strg.Format("%1.1f",Ymin + i*(Ymax-Ymin)/(Y_DivNumber-1));
		pDC->TextOut(pRectPlotPix->left - 5,
					 pRectPlotPix->bottom - (i*(pRectPlotPix->bottom-pRectPlotPix->top))/(Y_DivNumber-1),
					 Strg);
	}

	// Labels:
	if (pgDataLogger->Scopes[nscope].TimeSourceIndex >= 0){
		Strg.Format("%s[%s]",pgDataLogger->Variables[pgDataLogger->Scopes[nscope].TimeSourceIndex].VariableName,pgDataLogger->Variables[pgDataLogger->Scopes[nscope].TimeSourceIndex].VariableUnit);
	}
	else{
		Strg.Format("index");
	}
	pDC->SetTextAlign(TA_CENTER | TA_BASELINE);
	pDC->TextOut(pRectDCPix->left + (pRectDCPix->right-pRectDCPix->left)/2,
			pRectDCPix->bottom - 5,
			Strg);

	pDC->SelectObject(pTextFontRotated);
	Strg.Format("");
	for(i=0; i<pgDataLogger->Scopes[nscope].NumberOfInsertedVariables; ++i){
		j = pgDataLogger->Scopes[nscope].InsertedVariablesIndex[i];
		Strg.Format("%s %s[%s]",LPCTSTR(Strg),pgDataLogger->Variables[j].VariableName,pgDataLogger->Variables[j].VariableUnit);
	}
	pDC->SetTextAlign(TA_CENTER | TA_BASELINE);
	TextOutRotated(pDC, Strg, pRectDCPix->left + 5,
				 pRectDCPix->bottom + (pRectDCPix->top - pRectDCPix->bottom)/2,
				 90.0);
	// Restore previous drawing objects
	pDC->SelectObject(pOldPen);
	pDC->SelectObject(pOldBrush);
	pDC->SelectObject(pOldFont);

	return 1;
}


void GDataLogger::ScopeDrawPlotVariables(int nscope)
{
	CPen PenData;
	CPen *pPreviousPen;
	int i,nvar,ninsertedvar,queueindex;
	int Xpix, Ypix;
	double Xpixfloat, Ypixfloat;
	double X_DivScale; 
	double Y_DivScale; 
	double X_Offset; 
	double Y_Offset; 
	int X_DivNumber;
	int	Y_DivNumber;
	double Xmin, Xmax, Ymin, Ymax, X, Y;
	CRect *pRectPlotPix;
	CWnd *pWnd;
	CDC  *pDC;

	pWnd = pgDataLogger->Scopes[nscope].pWnd;
	pDC  = pgDataLogger->Scopes[nscope].pDC;

	X_DivScale = pgDataLogger->Scopes[nscope].Parameters.X_DivScale;
	Y_DivScale = pgDataLogger->Scopes[nscope].Parameters.Y_DivScale;
	X_Offset = pgDataLogger->Scopes[nscope].Parameters.X_Offset;
	Y_Offset = pgDataLogger->Scopes[nscope].Parameters.Y_Offset;
	X_DivNumber = pgDataLogger->Scopes[nscope].Parameters.X_DivNumber; 
	Y_DivNumber = pgDataLogger->Scopes[nscope].Parameters.Y_DivNumber; 
	pRectPlotPix = &pgDataLogger->Scopes[nscope].Parameters.RectPlotPix;

	Xmin = X_Offset - (X_DivNumber/2)* X_DivScale;
	Xmax = X_Offset + (X_DivNumber/2)* X_DivScale;
	Ymin = Y_Offset - (Y_DivNumber/2)* Y_DivScale;
	Ymax = Y_Offset + (Y_DivNumber/2)* Y_DivScale;

	pPreviousPen = (CPen*) pDC->GetCurrentPen(); 

	for(ninsertedvar=0;ninsertedvar<pgDataLogger->Scopes[nscope].NumberOfInsertedVariables;++ninsertedvar){
		// Get varindex:
		nvar = pgDataLogger->Scopes[nscope].InsertedVariablesIndex[ninsertedvar];
		// Pen
		PenData.CreatePen( PS_SOLID, 1, RGB(255,0,0));
		pDC->SelectObject(&PenData);
		// Empty queue:
		i = 0;
		while(gQUEUE_RequestReadIndex(&pgDataLogger->Variables[nvar].CircularQueueControl, QUEUE_DESTINATION_SCOPE, &queueindex)){
			Y = pgDataLogger->Variables[nvar].CircularQueue[queueindex];
			if (pgDataLogger->Scopes[nscope].TimeSourceIndex >= 0){
				X = pgDataLogger->Variables[pgDataLogger->Scopes[nscope].TimeSourceIndex].CircularQueue[queueindex];
			}
			else{
				break;
			}
			if ( (X>=Xmin) && (X<=Xmax) && (Y>=Ymin) && (Y<=Ymax) ){
				Xpixfloat = (X-Xmin)/(Xmax-Xmin)*(pRectPlotPix->right-pRectPlotPix->left);
				Ypixfloat = (Y-Ymin)/(Ymax-Ymin)*(pRectPlotPix->bottom-pRectPlotPix->top);
				Xpix = (int)(Xpixfloat);
				Ypix = (int)(Ypixfloat);
				if (pgDataLogger->Scopes[nscope].FirstPointPloted[ninsertedvar]){
					pDC->MoveTo(pRectPlotPix->left + pgDataLogger->Scopes[nscope].PreviousXpix[ninsertedvar], pRectPlotPix->bottom - pgDataLogger->Scopes[nscope].PreviousYpix[ninsertedvar]);
					pDC->LineTo(pRectPlotPix->left + Xpix, pRectPlotPix->bottom - Ypix);
				}
				pgDataLogger->Scopes[nscope].PreviousXpix[ninsertedvar] = Xpix;
				pgDataLogger->Scopes[nscope].PreviousYpix[ninsertedvar] = Ypix;
				pgDataLogger->Scopes[nscope].FirstPointPloted[ninsertedvar] = TRUE;
				++i;
			}
		}
	}

	pDC->SelectObject(pPreviousPen);
}

int GDataLogger::ScopeGetIndex(int nIDC)
{
	for(int n=0;n<pgDataLogger->m_NumberOfScopes;++n){
		if(pgDataLogger->Scopes[n].nIDC==nIDC){
			return(n);
		}
	}
	return(-1);
}
*/
