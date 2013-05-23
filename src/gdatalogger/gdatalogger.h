/*****************************************************************************
// File: GDataLogger.h
// Contents: Header for GDataLogger functions.
// Author: G. A. Borges.
*****************************************************************************/
#ifndef GDATALOGGER_H
#define GDATALOGGER_H

#ifdef __cplusplus
 extern "C" {
#endif 

#define GDATALOGGER_MAXVARIABLES 150

typedef struct{
	char VariableName[100];
	char VariableUnit[50];
	long int GMatlabDataFileIndex;
	int Nr;
	int Nc;
	double *CircularQueue;
	GQUEUECONTROL CircularQueueControl;
    int HasBeenWritten;
} GDATALOGGERVARIABLE,*PGDATALOGGERVARIABLE;

typedef struct{
	GDATALOGGERVARIABLE Variables[GDATALOGGER_MAXVARIABLES];
	int m_NumberOfVariables;
	GMATLABDATAFILECONFIG GMatlabDataFileConfig;
}GDATALOGGER,*PGDATALOGGER;

#define QUEUE_MAXDESTINATIONS	 4
#define QUEUE_DESTINATION_FILE   QUEUE_READ_GATE_0
#define QUEUE_DESTINATION_IPC    QUEUE_READ_GATE_1
#define QUEUE_DESTINATION_USER1  QUEUE_READ_GATE_2
#define QUEUE_DESTINATION_USER2  QUEUE_READ_GATE_3

#define GDATALOGGER_IPC_MAXQUEUESIZE		1000
#define GDATALOGGER_IPC_FLAGIDDLE			0
#define GDATALOGGER_IPC_FLAGREQUESTDATA		1
#define GDATALOGGER_IPC_FLAGDATAAVAILABLE	2
#define GDATALOGGER_IPC_FLAGNOTEXIST		3
#define GDATALOGGER_IPC_FLAGTIMEOUT			4

typedef struct {
	char VariableName[100];
	char VariableUnit[50];
	double QueueData[GDATALOGGER_IPC_MAXQUEUESIZE];
	int  QueueSize;
	int  Flag;
} GDATALOGGERIPC_SHM;

#if DATALOGGER_COMPILE_FOR_XENOMAI
	#define GDATALOGGER_IPC_STATEFILE "gdataloggershm"
#else
	#define GDATALOGGER_IPC_STATEFILE "/gdatalogger.shm"
#endif

int gDataLogger_Init(PGDATALOGGER pgDataLogger, char *filename, char *dirname);
int gDataLogger_DeclareVariable(PGDATALOGGER pgDataLogger, char *varname, char *varunit, int nrows, int ncols, int queuesize);
int gDataLogger_InsertVariable(PGDATALOGGER pgDataLogger, char *varname, double *varvalue);
int gDataLogger_IPC_RetrieveVariable(char *varname, char *varunit, double *pbuffer, int *bufferlen);
int gDataLogger_MatfileUpdate(PGDATALOGGER pgDataLogger);
int gDataLogger_IPCUpdate(PGDATALOGGER pgDataLogger);
int gDataLogger_Close(PGDATALOGGER pgDataLogger);

#ifdef __cplusplus
}
#endif 

#endif /* GDATALOGGER_H */
